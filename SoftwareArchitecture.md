# Legged Robots — Software Architecture

## Overview

The stack is built on top of **ROS + ros_control** and follows a clean separation between three layers:

```
[ Hardware Layer ]  ←→  [ Controller Layer ]  ←→  [ Planning Layer ]
  LeggedHW                LeggedController          MPC (OCS2)
  UnitreeHW               StateEstimateBase         WBC (QP solver)
  LeggedHWSim             KalmanFilterEstimate
```

All three layers communicate through **shared memory** (raw pointers wrapped in hardware interface handles) — no message passing, no copying.

---

## Package Map

| Package | Role |
|---------|------|
| `legged_common` | Shared hardware interface definitions (handles + interfaces) |
| `legged_hw` | Abstract base hardware class + control loop |
| `legged_gazebo` | Gazebo simulation hardware plugin |
| `legged_unitree_hw` | Real Unitree robot hardware driver |
| `legged_controllers` | Main ROS controller (MPC + WBC loop) |
| `legged_estimation` | State estimators (Kalman filter, from-topic) |
| `legged_interface` | OCS2 optimal control problem definition |
| `legged_wbc` | Whole-body controller (QP formulation) |

---

## 1. Hardware Layer

### Class Hierarchy

```
hardware_interface::RobotHW  (ros_control)
  └── LeggedHW               (legged_hw) — abstract base, registers all interfaces
        ├── UnitreeHW         (legged_unitree_hw) — UDP comms with Unitree SDK
        └── LeggedHWSim       (legged_gazebo) — Gazebo physics plugin
```

`LeggedHW` registers four hardware interfaces into the ros_control resource manager:

| Interface | Direction | What it carries |
|-----------|-----------|-----------------|
| `JointStateInterface` | HW → Controller | joint pos, vel, effort (read-only) |
| `HybridJointInterface` | Controller → HW | pos_des, vel_des, kp, kd, feedforward |
| `ImuSensorInterface` | HW → Controller | quaternion, ang_vel, lin_accel |
| `ContactSensorInterface` | HW → Controller | 4 foot contact booleans |

### How shared memory works

The hardware allocates the actual data buffers (e.g. `jointData_[i].posDes_`). It creates handles that hold **raw pointers** into those buffers and registers them. When the controller calls `getHandle()`, it gets a copy of the handle — same pointers, different object. Writing through the controller's copy directly updates the hardware's buffer.

```
Controller handle         Hardware buffer
  posDes_ ─────────────► jointData_[i].posDes_  ◄─── write() reads here → motor
  velDes_ ─────────────► jointData_[i].velDes_
  kp_     ─────────────► jointData_[i].kp_
  kd_     ─────────────► jointData_[i].kd_
  ff_     ─────────────► jointData_[i].ff_
```

### Control loop (`LeggedHWLoop`)

Runs at ~1 kHz in a real-time thread:

```
loop:
  hardware->read()           // pull sensor data into buffers
  controllerManager->update() // run LeggedController::update()
  hardware->write()          // push command buffers to motors
```

---

## 2. Controller Layer

### Class Hierarchy

```
controller_interface::MultiInterfaceController<HybridJointInterface,
                                               ImuSensorInterface,
                                               ContactSensorInterface>
  └── LeggedController          — main controller
        └── LeggedCheaterController  — uses external odometry instead of estimator
```

`LeggedController` holds:
- `vector<HybridJointHandle> hybridJointHandles_` — 12 joint command handles
- `ImuSensorHandle imuSensorHandle_`
- `vector<ContactSensorHandle> contactHandles_`
- `StateEstimateBase* stateEstimate_`
- `WbcBase* wbc_`
- `MPC_MRT_Interface* mpcMrtInterface_`

### `update()` pipeline (every 1ms)

```
1. Read hardware state
   hybridJointHandles_[i].getPosition() / getVelocity()
   imuSensorHandle_.getOrientation() / getAngularVelocity() / getLinearAcceleration()
   contactHandles_[i].isContact()

2. State estimation
   stateEstimate_->updateJointStates(pos, vel)
   stateEstimate_->updateContact(contactFlag)
   stateEstimate_->updateImu(quat, angVel, linAccel)
   measuredRbdState_ = stateEstimate_->update(time, period)
   currentObservation_.state = rbdConversions_->computeCentroidalState(measuredRbdState_)

3. Evaluate MPC policy (latest solution from async thread)
   mpcMrtInterface_->updatePolicy()
   mpcMrtInterface_->evaluatePolicy(time, state → optimizedState, optimizedInput, mode)

4. Whole-body control QP
   wbcTorques = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, mode, period)

5. Send commands to hardware
   hybridJointHandles_[j].setCommand(posDes, velDes, kp=0, kd=3, ff=wbcTorques[j])
```

---

## 3. State Estimation Layer

### Class Hierarchy

```
StateEstimateBase          (legged_estimation) — abstract
  ├── KalmanFilterEstimate  — fuses IMU + foot contact heights (EKF)
  └── FromTopicStateEstimate — subscribes to external odometry topic
```

`StateEstimateBase` provides three `update*()` methods that buffer incoming sensor data, and one pure virtual `update()` that produces the estimated state:

```
updateJointStates(pos, vel)  ─┐
updateContact(flags)          ├─► update(time, period) → measuredRbdState_
updateImu(quat, angVel, accel)┘
```

`KalmanFilterEstimate` fuses IMU accelerations with foot contact kinematics to estimate base position and velocity. `FromTopicStateEstimate` (used by `LeggedCheaterController`) just passes through external odometry.

---

## 4. Planning Layer

### MPC (OCS2 framework)

Runs in a **separate thread** at ~50 Hz, decoupled from the 1kHz control loop:

```
MPC thread:
  receive currentObservation_ (state, time, mode)
  solve optimal control problem over receding horizon
  publish policy (optimizedState, optimizedInput as functions of time)

Control thread (every 1ms):
  evaluatePolicy(t) → interpolate from latest MPC solution
```

`LeggedInterface` defines the OCS2 problem: cost function, system dynamics, constraints (friction cone, swing foot clearance, etc.).

### WBC (Whole-Body Controller)

```
WbcBase                  — abstract, formulates QP tasks
  ├── WeightedWbc         — soft constraints via weighted least-squares
  └── HierarchicalWbc     — hard priority ordering of tasks
```

Decision variables: `x = [joint_accelerations, contact_forces, joint_torques]`

Tasks formulated each cycle:
- Floating base equation of motion (equality constraint)
- Torque limits
- Friction cone (contact forces)
- Base acceleration tracking (from MPC)
- Swing leg tracking (from MPC)

Output: 12 joint torques → sent via `HybridJointHandle::setCommand(ff=torque)`

---

## 5. Full Data Flow (one control cycle)

```
┌─────────────────────────────────────────────────────────────────────┐
│  HARDWARE (UnitreeHW or LeggedHWSim)                                │
│                                                                     │
│  read():  encoders, IMU, foot sensors → jointData_[], imuData_[]   │
│  write(): jointData_[].posDes/velDes/kp/kd/ff → motor SDK / Gazebo │
└────────────────────┬──────────────────────────────▲────────────────┘
                     │ shared memory (raw pointers)  │
                     ▼                               │
┌─────────────────────────────────────────────────────────────────────┐
│  CONTROLLER (LeggedController::update @ 1kHz)                       │
│                                                                     │
│  [read handles] → [state estimation] → [MPC eval] → [WBC QP]       │
│                                                       │             │
│                                          torques[12] ─┤             │
│                                   setCommand(...)  ───┘             │
└──────────────────────────────────────────────────────┬──────────────┘
                                                       │ async
                                                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  MPC THREAD (~50 Hz)                                                │
│                                                                     │
│  currentObservation → OCS2 solver → policy(t)                      │
│  Controller polls policy each cycle via mpcMrtInterface_            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6. Key Design Choices

**Shared-memory handles** — No ROS topics between hardware and controller inside the same process. Hardware registers pointer-based handles; controller gets copies that share the same underlying memory. Zero-copy, deterministic timing.

**Async MPC** — The MPC solver runs in its own thread and can take >1ms. The controller always uses the latest available policy without blocking, ensuring the 1kHz loop is never stalled.

**Abstract hardware base** — `LeggedHW` lets the same controller code run on real hardware (`UnitreeHW`) and simulation (`LeggedHWSim`) without modification. Only the hardware driver changes.

**Abstract state estimator** — `StateEstimateBase` lets the controller swap between EKF estimation (`KalmanFilterEstimate`) and ground-truth odometry (`FromTopicStateEstimate`) via a single pointer.

**Abstract WBC** — `WbcBase` allows swapping between weighted (`WeightedWbc`) and hierarchical (`HierarchicalWbc`) task formulations without changing the controller.
