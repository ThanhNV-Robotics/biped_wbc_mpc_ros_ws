# Biped WBC-MPC — Software Architecture

## Overview

The stack is built on **ROS 2 (Jazzy) + ros2_control** and follows a clean separation between three layers:

```
[ Hardware Layer ]  ←→  [ Controller Layer ]  ←→  [ Planning Layer ]
  MujocoSystemInterface   LeggedController          MPC (OCS2)
  (future: LeggedHW)      StateEstimateBase         WBC (QP solver)
                          joint_state_broadcaster
                          imu_sensor_broadcaster
```

State and command data flow through **ros2_control's loaned interface handles** — no topic copying inside the control loop. The MPC solver runs asynchronously in its own thread and communicates with the controller via a thread-safe MRT interface.

---

## Package Map

| Package | Role | Status |
|---------|------|--------|
| `legged_common` | Shared types, ROS 2 hardware interface definitions | Active |
| `mujoco_ros2_control` | MuJoCo simulation hardware interface plugin | Active |
| `biped_description` | URDF, MJCF robot model, mesh assets | Active |
| `biped_centroidal_mpc` | Launch files, controller configs, gains | Active |
| `legged_controllers` | WBC + MPC ros2_control controller plugin | Planned |
| `legged_estimation` | State estimators (EKF, from-topic) | Planned |
| `legged_interface` | OCS2 optimal control problem definition | Planned |
| `legged_wbc` | Whole-body controller (QP formulation) | Planned |

---

## ros2_control Framework

ros2_control is the middleware that connects simulation/hardware to controllers. It defines two plugin types that run inside the same process with zero topic overhead.

### Hardware Interface Plugin (`hardware_interface::SystemInterface`)

A hardware plugin bridges the physical robot (or simulator) to the ros2_control resource manager. It must implement:

```
on_init()       — parse URDF hardware parameters, initialize hardware
on_activate()   — start communication/simulation threads
on_deactivate() — stop threads cleanly
read()          — copy sensor data into StateInterface buffers  (called each cycle)
write()         — read CommandInterface buffers and send to hardware (called each cycle)
export_state_interfaces()    — declare what state data is available (pos, vel, effort, IMU, …)
export_command_interfaces()  — declare what commands are accepted (position, velocity, effort)
```

The plugin is declared in the URDF `<ros2_control>` block and loaded dynamically via pluginlib:

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
    <param name="mujoco_model">path/to/model.xml</param>
    <param name="pd_gains_file">path/to/gains.yaml</param>
  </hardware>
  <joint name="left_hip_pitch_joint">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  ...
</ros2_control>
```

### Controller Plugin (`controller_interface::ControllerInterface`)

A controller plugin runs inside the controller_manager at the configured rate. It must implement:

```
on_init()          — declare parameters
on_configure()     — read parameters, set up internal state
on_activate()      — claim state/command interfaces from resource manager
on_deactivate()    — release interfaces
update(time, dt)   — main control loop: read state → compute commands → write commands
```

Controllers claim interfaces by name in `on_activate()`:

```cpp
// Claim state interfaces
for (auto& info : joint_names_) {
    state_interfaces_.emplace_back(get_node()->get_interface(info + "/position"));
    state_interfaces_.emplace_back(get_node()->get_interface(info + "/velocity"));
}
// Claim command interfaces
for (auto& info : joint_names_) {
    command_interfaces_.emplace_back(get_node()->get_interface(info + "/effort"));
}
```

### Resource Manager and Control Loop

The controller_manager orchestrates everything in a fixed-rate RT thread:

```
each cycle (2ms @ 500Hz):
  resource_manager.read(time, period)     ← hardware.read() for all hardware
  controller_manager.update(time, period) ← controller.update() for all active controllers
  resource_manager.write(time, period)    ← hardware.write() for all hardware
```

```
┌─────────────────────────────────────────────────────────────┐
│  controller_manager (500 Hz RT thread)                      │
│                                                             │
│  ┌─────────────────────┐     ┌──────────────────────────┐  │
│  │  ResourceManager    │     │  LeggedController        │  │
│  │                     │◄────│  .update(t, dt)          │  │
│  │  StateInterfaces[]  │────►│    read state handles    │  │
│  │  CommandInterfaces[]│◄────│    write command handles │  │
│  └──────────┬──────────┘     └──────────────────────────┘  │
│             │ loaned handles (zero-copy)                    │
└─────────────┼───────────────────────────────────────────────┘
              │
  ┌───────────▼───────────┐
  │  MujocoSystemInterface│
  │  .read()  ← physics   │
  │  .write() → actuators │
  └───────────────────────┘
```

---

## 1. Hardware Layer

### MujocoSystemInterface (`mujoco_ros2_control`)

Wraps the MuJoCo Simulate application as a ros2_control `SystemInterface` plugin.

**Threads:**
- `physics_thread_` — runs `PhysicsLoop()`, advances `mj_step()` at MuJoCo timestep (1000 Hz), publishes `/clock`
- `ui_thread_` — runs the GLFW rendering window (headless mode skips this)
- `clock_executor_thread_` (Humble only) — spins the minimal `mujoco_clock` node

**Node design:** A minimal `mujoco_clock` node (not a full feature node) is created solely to host the `/clock` publisher and receive plugin parameters. On Jazzy it shares the controller_manager's executor; on Humble it owns a `SingleThreadedExecutor` thread.

**PD impedance mode:** When velocity command interfaces are active, `write()` applies an internal PD law using gains loaded from `pd_gains_file`:
```
torque = Kp * (pos_des - pos) + Kd * (vel_des - vel)
```
This lets the robot hold a stable pose before any external controller is connected.

**Interfaces exported:**

| Interface | Type | Description |
|-----------|------|-------------|
| `<joint>/position` | State | Joint angle (rad) |
| `<joint>/velocity` | State | Joint velocity (rad/s) |
| `<joint>/effort` | State | Applied torque (Nm) |
| `<joint>/position` | Command | Target position |
| `<joint>/velocity` | Command | Target velocity (used for PD mode) |
| `<joint>/effort` | Command | Direct torque feedforward |
| `<imu>/orientation.{x,y,z,w}` | State | IMU quaternion |
| `<imu>/angular_velocity.{x,y,z}` | State | IMU gyro (rad/s) |
| `<imu>/linear_acceleration.{x,y,z}` | State | IMU accelerometer (m/s²) |

**Topics published:**
- `/clock` — sim time (required for `use_sim_time: true`)
- `/simulator/floating_base_state` (`nav_msgs/Odometry`) — free joint pose and twist

### Future: Real Hardware Interface

A `LeggedHW` plugin (to be written, modeled after `legged_robots/legged_hw`) will replace `MujocoSystemInterface` for real robot deployment. It communicates with the STM32 motor controller via UDP. The controller code is hardware-agnostic — only the URDF `<plugin>` tag changes.

---

## 2. Controller Layer

### Active Controllers

| Controller | Package | Purpose |
|------------|---------|---------|
| `joint_state_broadcaster` | `ros2_controllers` | Publishes `/joint_states` from state interfaces |
| `imu_sensor_broadcaster` | `ros2_controllers` | Publishes `/imu_sensor_broadcaster/imu` from IMU state interfaces |

### Planned: LeggedController

```
controller_interface::ControllerInterface
  └── LeggedController          — main WBC + MPC controller
        └── LeggedCheaterController — uses ground-truth odometry (sim only)
```

`LeggedController::update()` pipeline (every 2ms @ 500Hz):

```
1. Read hardware state
     joint position/velocity/effort from StateInterface handles
     IMU orientation/angular_velocity/linear_acceleration from StateInterface handles
     floating base from /simulator/floating_base_state (or estimator)

2. State estimation
     stateEstimate_->updateJointStates(pos, vel)
     stateEstimate_->updateImu(quat, angVel, linAccel)
     stateEstimate_->updateContact(contactFlags)
     measuredRbdState_ = stateEstimate_->update(time, period)
     currentObservation_.state = rbdConversions_->computeCentroidalState(measuredRbdState_)

3. Evaluate MPC policy (latest solution from async thread)
     mpcMrtInterface_->updatePolicy()
     mpcMrtInterface_->evaluatePolicy(time, state → optimizedState, optimizedInput, mode)

4. Whole-body control QP
     wbcTorques = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, mode, period)

5. Send commands to hardware
     command_interfaces_[j].set_value(wbcTorques[j])   // effort interface
```

---

## 3. State Estimation Layer

```
StateEstimateBase          — abstract base
  ├── KalmanFilterEstimate  — fuses IMU + foot contact kinematics (EKF)
  └── FromTopicStateEstimate — subscribes to /simulator/floating_base_state (sim ground truth)
```

`StateEstimateBase` buffers incoming sensor data and produces a rigid-body dynamics state:

```
updateJointStates(pos, vel)    ─┐
updateContact(flags)            ├─► update(time, period) → measuredRbdState_
updateImu(quat, angVel, accel) ─┘
```

`KalmanFilterEstimate` fuses IMU accelerations with foot contact kinematics to estimate base position and linear velocity. `FromTopicStateEstimate` (used by `LeggedCheaterController`) passes through the ground-truth odometry from `/simulator/floating_base_state`.

---

## 4. Planning Layer

### MPC (OCS2 Framework)

Runs in a **separate thread** at ~50 Hz, decoupled from the 500Hz control loop:

```
MPC thread (~50 Hz):
  receive currentObservation_ (state, time, mode)
  solve centroidal dynamics optimal control problem over receding horizon
  publish policy: (optimizedState(t), optimizedInput(t)) as time-parameterized functions

Control thread (every 2ms):
  mpcMrtInterface_->evaluatePolicy(t) → interpolate from latest MPC solution
```

`LeggedInterface` defines the OCS2 problem: cost function, centroidal dynamics, constraints (friction cone, swing foot clearance, contact schedule, joint limits).

### MPC Reference: `/legged_robot_mpc_target`

The MPC cost function tracks a **TargetTrajectories** reference, delivered over the topic `/legged_robot_mpc_target` (type `ocs2_msgs/msg/MpcTargetTrajectories`):

```
time_trajectory:   float64[]       # timestamps of the waypoints
state_trajectory:  MpcState[]      # desired centroidal state at each waypoint
  └─ value: float32[24]            # same 24-dim layout as /legged_robot_mpc_observation
input_trajectory:  MpcInput[]      # desired input (zeros — not used in the reference)
  └─ value: float32[18]
```

**State trajectory layout** (`value[i]` for each waypoint):

| Index | Meaning |
|-------|---------|
| `[0]`–`[2]` | Target linear momentum / mass ≈ CoM linear velocity (vx, vy, vz) in world frame |
| `[3]`–`[5]` | Target angular momentum / mass ≈ angular velocity (wx, wy, wz) in world frame |
| `[6]`–`[8]` | Target base position (px, py, pz) |
| `[9]`–`[11]` | Target base orientation ZYX Euler (yaw, pitch=0, roll=0) |
| `[12]`–`[23]` | Target joint angles (default standing configuration) |

**Reference pipeline:**

```
/humanoid/walking_velocity_command   (25 Hz from GUI)
  cmd = [vx_norm, vy_norm, pelvis_height_m, omega_z_norm]
         │         │         │                │
         × max_vx  × max_vy  passthrough      × max_omega_z
         └─────────────────── low-pass filter ────────────────────┘
                    ↓
         TargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories()
           builds 3-waypoint trajectory:
             t0           → current momentum + current base pose
             t0 + 0.7·T  → integrated intermediate pose
             t0 + T       → integrated final pose
                    ↓
         TargetTrajectoriesRosPublisher → /legged_robot_mpc_target
                    ↓
         RosReferenceManager::preSolverRun() → setTargetTrajectories()
                    ↓
         OCS2 MPC cost function
```

**Initial reference** (set at `on_activate()`): standing still at the current base pose and default joint angles, applied directly via `setTargetTrajectories()` — not through the topic — to avoid a race with the MPC thread start.

---

### WBC (Whole-Body Controller)

```
WbcBase
  ├── WeightedWbc      — soft constraints via weighted least-squares
  └── HierarchicalWbc  — hard priority task ordering
```

Decision variables per cycle: `x = [joint_accelerations(12), contact_forces(12), joint_torques(12)]`

Tasks formulated each cycle:
- Floating base equation of motion (equality)
- Torque limits (inequality)
- Friction cone for contact forces (inequality)
- Base linear/angular acceleration tracking from MPC (equality)
- Swing foot acceleration tracking from MPC (equality)

Output: 12 joint torques → written to effort command interfaces.

---

## 5. Full Data Flow (one control cycle @ 500Hz)

```
┌──────────────────────────────────────────────────────────────────────┐
│  MujocoSystemInterface                                               │
│                                                                      │
│  physics_thread_: mj_step() @ 1000Hz                                │
│    → joint pos/vel/effort, IMU, floating base into mj_data_          │
│                                                                      │
│  read(t, dt):  mj_data_ → StateInterface buffers                    │
│  write(t, dt): CommandInterface buffers → mj_data_.ctrl[]           │
│                + internal PD law if velocity commands active         │
└──────────┬──────────────────────────────────▲────────────────────────┘
           │ StateInterface (loaned, zero-copy) │ CommandInterface
           ▼                                   │
┌──────────────────────────────────────────────────────────────────────┐
│  LeggedController::update() @ 500Hz                                  │
│                                                                      │
│  [read state] → [estimation] → [MPC eval] → [WBC QP] → [write cmd] │
└─────────────────────────────────────────────────────────┬────────────┘
                                                          │ async
                                                          ▼
┌──────────────────────────────────────────────────────────────────────┐
│  MPC Thread (~50Hz)                                                  │
│                                                                      │
│  currentObservation → OCS2 centroidal MPC → policy(t)               │
│  Controller polls via mpcMrtInterface_ each cycle (non-blocking)     │
└──────────────────────────────────────────────────────────────────────┘

Topics visible externally:
  /clock                                ← sim time
  /joint_states                         ← joint_state_broadcaster
  /imu_sensor_broadcaster/imu           ← imu_sensor_broadcaster
  /simulator/floating_base_state        ← MujocoSystemInterface (ground-truth odometry)
  /tf                                   ← robot_state_publisher
  /legged_robot_mpc_observation         ← LeggedController: current centroidal state (24-dim)
  /legged_robot_mpc_target              ← LeggedController: MPC reference trajectory (3 waypoints)
  /humanoid/walking_velocity_command    ← GUI: [vx_norm, vy_norm, height_m, omega_z_norm]
```

---

## 6. Launch Architecture

```
mujoco_sim.launch.py
  ├── robot_state_publisher        — publishes URDF to /robot_description + /tf
  ├── ros2_control_node            — controller_manager + MujocoSystemInterface
  │     parameters: mujoco_controllers.yaml (update_rate: 500Hz)
  └── spawner                      — activates joint_state_broadcaster + imu_sensor_broadcaster

Launch arguments:
  headless      (default: false)  — run MuJoCo without GUI window
  pd_gains_file (default: biped_centroidal_mpc/config/pd_impedance_controller.yaml)
```

---

## 7. Key Design Choices

**ros2_control plugin model** — Hardware and controllers are dynamically loaded plugins. The same `LeggedController` plugin runs against `MujocoSystemInterface` in simulation or a future `LeggedHW` plugin on real hardware. Only the URDF `<plugin>` tag changes.

**Loaned interface handles** — ros2_control's `StateInterface` and `CommandInterface` use direct memory references (not topic messages) between hardware and controllers inside the controller_manager process. Zero-copy, deterministic timing at 500Hz.

**Minimal clock node** — `MujocoSystemInterface` creates a single `mujoco_clock` node solely to publish `/clock` and receive plugin parameters. No services, no monitoring publishers. On Jazzy it shares the CM executor; on Humble it owns a dedicated thread.

**Internal PD hold mode** — Before any external controller connects, `MujocoSystemInterface` applies a PD impedance law using gains from `pd_gains_file`. This keeps the robot in a stable pose without requiring a controller to be active.

**Async MPC** — The MPC solver runs in its own thread and may take >2ms. The controller always uses the latest available policy without blocking, ensuring the 500Hz loop is never stalled.

**Abstract hardware base** — `SystemInterface` lets the same controller code run in MuJoCo simulation and on real hardware without modification.

**Abstract state estimator** — `StateEstimateBase` allows swapping between EKF (`KalmanFilterEstimate`) and ground-truth (`FromTopicStateEstimate`) via a single pointer.
