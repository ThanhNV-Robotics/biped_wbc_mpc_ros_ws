# mujoco_ros2_control

A ROS 2 ros2_control system interface for the [MuJoCo Simulator](https://mujoco.readthedocs.io/en/stable/overview.html),
extended with a **HybridJoint PD impedance mode**, **foot contact sensors**, and an optional **UDP bridge** for
zero-latency external controllers.

Originally written for NASA Johnson's [iMETRO facility](https://ntrs.nasa.gov/citations/20230015485).
Parts are based on the MoveIt [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) package.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Threading Model & Data Safety](#2-threading-model--data-safety)
3. [Hardware Interface Setup](#3-hardware-interface-setup)
   - [Plugin Parameters](#31-plugin-parameters)
   - [Joint Control Modes](#32-joint-control-modes)
   - [HybridJoint PD Impedance Mode](#33-hybridjoint-pd-impedance-mode)
   - [Contact Sensors](#34-contact-sensors)
   - [IMU Sensors](#35-imu-sensors)
   - [Force-Torque Sensors](#36-force-torque-sensors)
   - [Cameras & Lidar](#37-cameras--lidar)
4. [UDP Bridge](#4-udp-bridge)
5. [PD Gains File](#5-pd-gains-file)
6. [Topics & Services](#6-topics--services)
7. [Debugging](#7-debugging)
8. [URDF Conversion](#8-urdf-conversion)

---

## 1. Architecture Overview

The stack follows a clean three-layer separation that mirrors the original ROS 1 `legged_robots` design:

```
[ Hardware Layer ]  ←→  [ Controller Layer ]  ←→  [ Planning Layer ]
  MujocoSystemInterface   LeggedController          MPC (OCS2)
  (mujoco_ros2_control)   (legged_controllers)      WBC (QP solver)
```

All three layers communicate through **shared memory** — the same `double*` pointers, wrapped in
ros2_control `CommandInterface` / `StateInterface` objects on the hardware side and
`LoanedCommandInterface` / `LoanedStateInterface` handles on the controller side.
No copying, no ROS topics between layers.

### Full data flow (one 1 kHz control cycle)

```
┌──────────────────────────────────────────────────────────────────────┐
│  MujocoSystemInterface  (hardware layer)                             │
│                                                                      │
│  read():                                                             │
│    mj_data_control_ → joint pos/vel/effort (state interfaces)       │
│    mj_data_control_ → IMU sensordata (orientation/gyro/accel)       │
│    mj_data_control_->contact[] → foot contact booleans              │
│                                                                      │
│  write():                                                            │
│    pos_des, vel_des, kp, kd, ff (command interfaces)                │
│    → tau = kp*(pos_des-q) + kd*(vel_des-qd) + ff                   │
│    → mj_data_control_->ctrl[]                                        │
└──────────────────┬───────────────────────────────▲───────────────────┘
                   │ controller_manager loans       │
                   │ LoanedStateInterface /         │
                   │ LoanedCommandInterface         │
                   ▼                               │
┌──────────────────────────────────────────────────────────────────────┐
│  LeggedController  (controller layer, ~1 kHz)                        │
│                                                                      │
│  [HybridJointHandle × 12]  →  read pos/vel, write pos/vel/kp/kd/ff  │
│  [ContactSensorHandle × 4] →  isContact()                           │
│  [ImuSensorHandle]         →  orientation / angVel / linAccel        │
│                                                                      │
│  update() pipeline:                                                  │
│    1. stateEstimate_->update()        (EKF or ground-truth)          │
│    2. mpcMrtInterface_->evaluatePolicy()  (async, never blocks)      │
│    3. wbc_->update()                  (QP, ~0.3 ms)                  │
│    4. handle.setCommand(pos, vel, 0, 3, wbcTorque[j])               │
└──────────────────────────────────────────────┬───────────────────────┘
                                               │ async ~50 Hz
                                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│  MPC thread  (OCS2, ~50 Hz)                                          │
│    currentObservation → OCS2 solver → policy(t)                     │
└──────────────────────────────────────────────────────────────────────┘
```

### Package map

| Package | Role |
|---------|------|
| `mujoco_ros2_control` | Hardware system interface — exports interfaces, runs physics loop |
| `legged_common` | Shared handle types: `HybridJointHandle`, `ContactSensorHandle` |
| `legged_controllers` | Main ROS 2 controller: MPC + WBC update loop |
| `legged_estimation` | State estimators (Kalman filter, from-topic) |
| `legged_interface` | OCS2 optimal control problem definition |
| `legged_wbc` | Whole-body controller (QP) |

---

## 2. Threading Model & Data Safety

Three threads run concurrently inside `MujocoSystemInterface`:

```
ros2_control thread (~1 kHz)     Physics thread (real-time)      Viewer / UI thread
  read()                           PhysicsLoop()                   GLFW render loop
  write()                            mju_copy ctrl  ─┐
                                     mj_step()        │  lock-free (expensive)
                                     mj_copyData  ────┘
```

### `mj_data_control_` double-buffer

`mj_data_` is the **live** physics state owned by the physics thread.
`mj_data_control_` is a **copy** used by `read()` and `write()`.
This decouples the 1 kHz control loop from the physics step timing.

```
Physics thread                       ros2_control thread
──────────────────────────           ──────────────────────────
lock(control_data_mutex_)            lock(control_data_mutex_)
mju_copy(ctrl ← mj_data_control_)   read sensor data ← mj_data_control_
unlock                               unlock
mj_step()          ← lock-free       ...
lock(control_data_mutex_)
mj_copyData(mj_data_control_ ← mj_data_)
unlock
```

**Rule:** `mj_step()` is never called while holding `control_data_mutex_`.
The mutex is only held for the brief copy operations before and after the step.
`sim_mutex_` (the viewer's recursive mutex) is kept entirely separate to avoid priority inversion.

---

## 3. Hardware Interface Setup

### 3.1 Plugin Parameters

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>

    <!-- Path to the MJCF model file (required unless mujoco_model_topic is used) -->
    <param name="mujoco_model">$(find my_description)/scene.xml</param>

    <!-- Subscribe to a topic for the MJCF instead of a file path -->
    <param name="mujoco_model_topic">/mujoco_robot_description</param>

    <!-- Run without the Simulate App GUI window -->
    <param name="headless">false</param>

    <!-- Run faster or slower than real time (>1 = faster). Ignored if <0. -->
    <param name="sim_speed_factor">1.0</param>

    <!-- Apply a named keyframe from the MJCF at startup -->
    <param name="initial_keyframe">home</param>

    <!-- Load start state from a saved 'Copy state' XML file -->
    <param name="override_start_position_file">$(find my_pkg)/start.xml</param>

    <!-- Path to YAML file with PD gains for internal hold-position mode (see section 5) -->
    <param name="pd_gains_file">$(find my_pkg)/config/pd_gains.yaml</param>

    <!-- Publish floating-base odometry (requires a MuJoCo free joint) -->
    <param name="odom_free_joint_name">floating_base</param>
    <param name="odom_topic">/simulator/floating_base_state</param>

    <!-- Enable the UDP bridge to an external controller (see section 4) -->
    <param name="use_udp">false</param>
    <param name="udp_controller_ip">127.0.0.1</param>
    <param name="udp_state_port">9870</param>
    <param name="udp_cmd_port">9871</param>
  </hardware>
  ...
</ros2_control>
```

### 3.2 Joint Control Modes

Each joint can operate in one of four exclusive modes, selected via `perform_command_mode_switch()`:

| Mode | Command interfaces claimed | What `write()` does |
|------|---------------------------|---------------------|
| **position** | `<joint>/position` | `ctrl = pos_cmd` |
| **velocity** | `<joint>/velocity` | `ctrl = vel_cmd` |
| **effort** | `<joint>/effort` | `ctrl = effort_cmd` |
| **PD impedance** | `<joint>/position` + `<joint>/velocity` + `<joint>/feedforward_effort` | `ctrl = kp*(pos_des-q) + kd*(vel_des-qd) + ff` |

The PD impedance mode is activated automatically when a controller claims both
`position`/`velocity` **and** `feedforward_effort` for the same joint.

**Compatibility with MuJoCo actuator types:**

| | `position` actuator | `velocity` actuator | `motor` / `general` |
|---|---|---|---|
| `position` cmd | Native | Via PID | Via PID |
| `velocity` cmd | Not supported | Native | Via PID |
| `effort` cmd | Not supported | Not supported | Native |
| **PD impedance** | **Native** | Not supported | **Native** |

> `torque` and `force` command/state interfaces are aliases for `effort` and map to the same data.

### 3.3 HybridJoint PD Impedance Mode

This is the central control mode for legged locomotion, directly mirroring the ROS 1
`HybridJointInterface` / `HybridJointHandle` pattern from `legged_common`.

#### What it provides

Each joint exposes **five command interfaces** and **two state interfaces**:

```
Command interfaces (written by the controller each cycle):
  <joint>/position          — desired position   (pos_des)
  <joint>/velocity          — desired velocity   (vel_des)
  <joint>/kp                — proportional gain  (variable, per-cycle)
  <joint>/kd                — derivative gain    (variable, per-cycle)
  <joint>/feedforward_effort — feedforward torque (ff)

State interfaces (written by hardware from MuJoCo physics):
  <joint>/position          — measured joint angle
  <joint>/velocity          — measured joint velocity
```

The torque applied each physics step:

```
tau = kp * (pos_des - q) + kd * (vel_des - qd) + ff
```

#### Variable gains (HybridJoint pattern)

`kp` and `kd` are **commanded per control cycle** by the controller — not fixed at startup.
This lets the WBC vary impedance dynamically:

| Mode | kp | kd | ff |
|------|----|----|-----|
| Stance (stiff) | 350 | 3 | WBC torque |
| Swing (compliant) | 0 | 3 | WBC torque |
| Pure torque control | 0 | 0 | WBC torque |

If the controller writes NaN (or never writes), the YAML defaults (`pd_gains_file`) are used as fallback.
If no `pd_gains_file` is set, gains default to 0.

At activation, if `pd_gains_file` is configured, the hardware automatically enters **internal
hold-position mode** (PD impedance with YAML gains, position seeded from current physics state)
without requiring any controller to be running.

#### URDF declaration

```xml
<joint name="left_hip_pitch_joint">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="feedforward_effort"/>
  <!-- kp and kd are always exported by the hardware; no declaration needed in URDF -->
  <state_interface name="position">
    <param name="initial_value">0.0</param>
  </state_interface>
  <state_interface name="velocity"/>
</joint>
```

> `kp` and `kd` command interfaces are exported automatically for every non-passive actuator —
> no `<command_interface name="kp"/>` entry is required in the URDF.

#### Controller side (legged_common)

```cpp
// on_activate() — build one handle per joint from the loaned interfaces:
hybridJointHandles_.emplace_back(
    "left_hip_pitch_joint",
    state_interfaces_[pos_idx], state_interfaces_[vel_idx],
    command_interfaces_[pos_idx], command_interfaces_[vel_idx],
    command_interfaces_[kp_idx],  command_interfaces_[kd_idx],
    command_interfaces_[ff_idx]);

// update() — identical call site to original ROS 1:
hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0.0, 3.0, wbcTorques(j));
```

### 3.4 Contact Sensors

Foot contact is detected by scanning MuJoCo's `mjData::contact` array each `read()` cycle.
A contact sensor registers a body by name; if that body appears in any active contact pair,
`contact_state` is set to `1.0` (else `0.0`).

#### URDF declaration

```xml
<sensor name="LF_FOOT">
  <state_interface name="contact"/>
  <param name="mujoco_type">contact</param>
  <!-- Name of the MuJoCo body to watch for contacts.
       Defaults to the sensor name if omitted. -->
  <param name="mujoco_body_name">LF_FOOT</param>
</sensor>
```

State interface exported: `LF_FOOT/contact` (1.0 = touching, 0.0 = airborne).

#### Controller side (legged_common)

```cpp
// on_activate():
contactHandles_.emplace_back("LF_FOOT", state_interfaces_[contact_idx]);

// update():
bool lf_contact = contactHandles_[0].isContact();
stateEstimate_->updateContact({lf_contact, rf_contact, lh_contact, rh_contact});
```

### 3.5 IMU Sensors

MuJoCo models an IMU as three separate sensors. Name them with the default suffixes (`_quat`,
`_gyro`, `_accel`) unless overridden.

**MJCF:**
```xml
<sensor>
  <framequat name="torso_quat" objtype="site" objname="imu_site"/>
  <gyro      name="torso_gyro" site="imu_site"/>
  <accelerometer name="torso_accel" site="imu_site"/>
</sensor>
```

**URDF ros2_control:**
```xml
<sensor name="torso_imu">
  <param name="mujoco_type">imu</param>
  <param name="mujoco_sensor_name">torso</param>
  <state_interface name="orientation.x"/>
  <state_interface name="orientation.y"/>
  <state_interface name="orientation.z"/>
  <state_interface name="orientation.w"/>
  <state_interface name="angular_velocity.x"/>
  <state_interface name="angular_velocity.y"/>
  <state_interface name="angular_velocity.z"/>
  <state_interface name="linear_acceleration.x"/>
  <state_interface name="linear_acceleration.y"/>
  <state_interface name="linear_acceleration.z"/>
</sensor>
```

Optional suffix overrides: `orientation_mjcf_suffix`, `angular_velocity_mjcf_suffix`,
`linear_acceleration_mjcf_suffix` (defaults: `_quat`, `_gyro`, `_accel`).

### 3.6 Force-Torque Sensors

**MJCF:**
```xml
<sensor>
  <force  name="ankle_fts_force"  site="ft_frame"/>
  <torque name="ankle_fts_torque" site="ft_frame"/>
</sensor>
```

**URDF ros2_control:**
```xml
<sensor name="ankle_fts">
  <param name="mujoco_type">fts</param>
  <param name="mujoco_sensor_name">ankle_fts</param>
  <state_interface name="force.x"/>
  <state_interface name="force.y"/>
  <state_interface name="force.z"/>
  <state_interface name="torque.x"/>
  <state_interface name="torque.y"/>
  <state_interface name="torque.z"/>
</sensor>
```

Optional: `force_mjcf_suffix` (default `_force`), `torque_mjcf_suffix` (default `_torque`).

### 3.7 Cameras & Lidar

Any `<camera>` in the MJCF is automatically published as RGB-D + CameraInfo ROS topics.
Lidar is implemented by wrapping sets of `<rangefinder>` sensors.
See the original upstream documentation or [docs/TOOLS.md](docs/TOOLS.md) for full details.

---

## 4. UDP Bridge

When enabled, `MujocoSystemInterface` sends a full robot-state snapshot over UDP on every
`read()` and applies the most recently received `JointCommandPacket` in `write()`, bypassing
the ros2_control command interfaces entirely.

### Packet formats (`udp_bridge.hpp`)

**State packet (sim → controller, every `read()`):**

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | `steady_clock` nanoseconds |
| `num_joints` | `uint32_t` | Number of joints in this packet |
| `joint_pos[32]` | `float[32]` | Joint positions (rad) |
| `joint_vel[32]` | `float[32]` | Joint velocities (rad/s) |
| `joint_torque[32]` | `float[32]` | Applied torques (Nm) |
| `imu_quat[4]` | `float[4]` | Quaternion (w, x, y, z) |
| `imu_gyro[3]` | `float[3]` | Angular velocity (rad/s) |
| `imu_accel[3]` | `float[3]` | Linear acceleration (m/s²) |

**Command packet (controller → sim, applied in `write()`):**

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_ns` | `uint64_t` | `steady_clock` nanoseconds |
| `num_joints` | `uint32_t` | Number of joints |
| `pos_des[32]` | `float[32]` | Desired positions |
| `vel_des[32]` | `float[32]` | Desired velocities |
| `kp[32]` | `float[32]` | Proportional gains |
| `kd[32]` | `float[32]` | Derivative gains |
| `torque_ff[32]` | `float[32]` | Feedforward torques |

The command applied per joint: `tau = kp*(pos_des - q) + kd*(vel_des - qd) + torque_ff`.

**Timeout:** if no packet arrives within 50 ms, `write()` falls back to the ros2_control
command interfaces automatically.

### Enable in URDF

```xml
<hardware>
  <param name="use_udp">true</param>
  <param name="udp_controller_ip">127.0.0.1</param>
  <param name="udp_state_port">9870</param>   <!-- sim sends state here -->
  <param name="udp_cmd_port">9871</param>     <!-- sim receives commands here -->
</hardware>
```

---

## 5. PD Gains File

A YAML file loaded at runtime via the `pd_gains_file` ROS 2 parameter.
Expected format (either top-level or under `/**:ros__parameters:`):

```yaml
pd_gains:
  kp:
    left_hip_pitch_joint:  350.0
    left_hip_roll_joint:   300.0
    left_hip_yaw_joint:    150.0
    left_knee_pitch_joint: 350.0
    left_ankle_roll_joint:  50.0
    left_ankle_pitch_joint: 50.0
    # ... right leg joints
  kd:
    left_hip_pitch_joint:   1.5
    left_hip_roll_joint:    1.5
    # ...
```

These gains serve two purposes:
1. **Internal hold-position mode** — used automatically between hardware activation and the
   moment a controller takes over, preventing the robot from collapsing.
2. **Fallback gains** — if `kp`/`kd` command interfaces are NaN (controller hasn't written yet),
   `write()` uses these values instead of 0.

Pass the file path from the launch file:

```python
Node(
    package="mujoco_ros2_control",
    executable="ros2_control_node",
    parameters=[{"pd_gains_file": "/path/to/pd_gains.yaml"}],
)
```

---

## 6. Topics & Services

### Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mujoco_actuators_states` | `sensor_msgs/msg/JointState` | All MuJoCo actuator states (pos, vel, effort) |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `<odom_topic>` | `nav_msgs/msg/Odometry` | Floating-base odometry (if `odom_free_joint_name` set) |

### Services

All services are under the node namespace (e.g. `/ros2_control_node/`).

**`~/set_pause`** (`mujoco_ros2_control_msgs/srv/SetPause`)
```bash
ros2 service call ~/set_pause mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"
```

**`~/reset_world`** (`mujoco_ros2_control_msgs/srv/ResetWorld`)
```bash
# Reset to startup state
ros2 service call ~/reset_world mujoco_ros2_control_msgs/srv/ResetWorld "{}"
# Reset to a named MJCF keyframe
ros2 service call ~/reset_world mujoco_ros2_control_msgs/srv/ResetWorld "{keyframe: 'home'}"
```

> Deactivate active controllers before calling `reset_world` to avoid the robot snapping back.

**`~/step_simulation`** (`mujoco_ros2_control_msgs/srv/StepSimulation`)

Advances a **paused** simulation by exactly N physics steps. Blocks until done.
```bash
ros2 service call ~/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 100}"
```

---

## 7. Debugging

### Pause / resume

Press **Space** in the Simulate window or use the `~/set_pause` service.

### Single-stepping

While paused, press the **right arrow key** (`→`) to step one physics step at a time.
Alternatively, use `~/step_simulation` for programmatic control:

```bash
# Pause first, then step-by-step:
ros2 service call ~/set_pause      mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"
ros2 service call ~/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 1}"
```

### Inspecting command/state values

Enable the commented-out `RCLCPP_INFO_THROTTLE` blocks in `write()` to log PD impedance
values per joint at 1 Hz:

```cpp
// In mujoco_system_interface.cpp, write(), PD impedance branch:
// Uncomment the DEBUG block to print per-joint torque breakdown.
```

---

## 8. URDF Conversion

MuJoCo does not support the full URDF/xacro feature set. Convert your robot description to MJCF:

```bash
# Using the bundled conversion script:
ros2 run mujoco_ros2_control make_mjcf_from_robot_description.py \
    --urdf my_robot.urdf --output scene.xml
```

See [docs/TOOLS.md](docs/TOOLS.md) for full documentation of the conversion tooling and known
limitations.
