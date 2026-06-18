# legged_controllers

ROS 2 control plugin implementing a Whole-Body Control + Model Predictive Control (WBC-MPC) locomotion controller for the 12-DOF biped robot.

---

## Architecture overview

```
Controller Manager
    │
    ├─ on_init()        ← declare parameters, load file paths
    ├─ on_configure()   ← build OCS2 problem, MPC, state estimator, WBC
    ├─ on_activate()    ← initialize observation, start MPC thread
    │
    └─ update() @ 500 Hz
           ├─ updateStateEstimation()   reads joints + IMU → centroidal state
           ├─ MPC_MRT_Interface         query latest MPC policy
           ├─ WeightedWbc               solve WBC QP for joint torques
           ├─ SafetyChecker             abort if unsafe
           └─ write joint commands      position + velocity + effort
```

The controller is built on top of the [OCS2](https://leggedrobotics.github.io/ocs2/) framework.  
The hardware backend is either **MuJoCo** (simulation) or **LeggedHW** (real robot).

---

## Plugin loading

`pluginlib` discovers the controller at runtime from the plugin description file.

**`legged_controllers_plugins.xml`**
```xml
<library path="legged_controllers">
  <class name="legged/LeggedController"
         type="legged::LeggedController"
         base_class_type="controller_interface::ControllerInterface"/>
  <class name="legged/LeggedCheaterController"
         type="legged::LeggedCheaterController"
         base_class_type="controller_interface::ControllerInterface"/>
</library>
```

**`config/main_controllers.yaml`** — registers the plugin with the Controller Manager:
```yaml
controller_manager:
  ros__parameters:
    legged_controller:
      type: legged/LeggedController
```

**Spawning the controller:**
```bash
ros2 run controller_manager spawner legged_controller \
  --param-file /path/to/legged_controller_params.yaml
```

The spawner calls the Controller Manager, which looks up `legged/LeggedController` via pluginlib,
loads the shared library, and triggers the lifecycle: `on_init` → `on_configure` → `on_activate`.

---

## Input files

Three file paths are **required** parameters (see [ROS 2 parameters](#ros-2-parameters)):

### `urdf_file` — Robot URDF

Path to the expanded URDF (or the xacro source).  
The OCS2 `LeggedInterface` parses it with Pinocchio to build the rigid-body model used by the MPC and state estimator.

Default location: `biped_description/urdf/biped_robot_12dof.urdf`

### `task_file` — MPC configuration (`task.info`)

Defines the entire optimal control problem. Key sections:

| Section | Purpose |
|---|---|
| `centroidalModelType` | `0` = Full centroidal dynamics |
| `model_settings.jointNames` | 12 actuated joints included in the MPC model |
| `model_settings.contactNames3DoF` | Foot frames used by the Kalman filter |
| `multiple_shooting` | SQP solver settings (`dt`, `nThreads`, `sqpIteration`, …) |
| `mpc` | `timeHorizon`, `mpcDesiredFrequency` (80 Hz), `mrtDesiredFrequency` (500 Hz) |
| `Q` / `R` / `Q_final` | State and input cost matrices |
| `contacts` | Friction cone, contact rectangle, soft-constraint parameters |
| `jointLimits` | Joint limit soft-constraint parameters |
| `collision_constraint` | Knee/ankle collision spheres |
| `kalmanFilter` | Process and sensor noise covariances for the linear Kalman filter |
| `swing_trajectory_config` | Foot lift height, liftoff/touchdown velocities |

Default location: `legged_controllers/config/mpc/task.info`

### `reference_file` — Standing reference (`reference.info`)

Defines the nominal standing posture and contact schedule used as the MPC reference trajectory.

Default location: `legged_controllers/config/mpc/reference.info`

---

## ROS 2 parameters

All parameters are declared in `on_init()` and can be overridden via a `--param-file` YAML.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `urdf_file` | `string` | `""` (required) | Path to the robot URDF |
| `task_file` | `string` | `""` (required) | Path to `task.info` |
| `reference_file` | `string` | `""` (required) | Path to `reference.info` |
| `joints` | `string[]` | 12 biped joints | Joint names matching the URDF `<joint>` tags |
| `imu_name` | `string` | `"torso_imu"` | IMU sensor name matching the URDF `<sensor>` tag |

**Example parameter YAML** (must use the nested structure below — flat dot-notation will not work):
```yaml
legged_controller:
  ros__parameters:
    urdf_file:      /path/to/biped_robot_12dof.urdf
    task_file:      /path/to/config/mpc/task.info
    reference_file: /path/to/config/mpc/reference.info
```

> **Why a temporary file?** `launch_ros` wraps Python dict parameters under `/**:ros__parameters:`
> with dot-flattened keys. The controller node cannot match them. The launch files write the YAML
> manually with `tempfile` + `yaml.dump()` and pass it via `--param-file`.

---

## Hardware interfaces

Declared in `command_interface_configuration()` and `state_interface_configuration()`.  
Names must exactly match the `<command_interface>` / `<state_interface>` tags in the URDF.

### Command interfaces (written by the controller)

For each of the 12 joints:
- `<joint_name>/position` — desired joint angle (PD target)
- `<joint_name>/velocity` — desired joint velocity (PD target)
- `<joint_name>/effort`   — feedforward torque from WBC

Requires `use_pid:=true` when expanding the URDF with xacro.

### State interfaces (read by the controller)

For each of the 12 joints:
- `<joint_name>/position` — measured joint angle
- `<joint_name>/velocity` — measured joint velocity
- `<joint_name>/effort`   — measured joint torque

IMU (`torso_imu` by default):
- `torso_imu/orientation.{x,y,z,w}`
- `torso_imu/angular_velocity.{x,y,z}`
- `torso_imu/linear_acceleration.{x,y,z}`

---

## Lifecycle methods

### `on_init()`
Declares and validates all ROS 2 parameters. Returns `FAILURE` if any required path is empty.

### `on_configure()` *(TODO — step 3)*
Runs all one-time setup in order:
1. `setupLeggedInterface()` — builds the OCS2 optimal control problem via Pinocchio
2. `setupMpc()` — creates `SqpMpc`, attaches `GaitReceiver` and `RosReferenceManager`
3. `setupMrt()` — creates `MPC_MRT_Interface`, initializes rollout (does **not** start the thread)
4. `setupStateEstimate()` — creates `KalmanFilterEstimate` on a dedicated `rclcpp::Node`
5. Setup `WeightedWbc` and `SafetyChecker`

### `on_activate()` *(TODO — step 3)*
1. Reads current joint positions to seed the initial observation
2. Calls `mpcMrtInterface_->advanceMpc()` in a loop until the first policy is received
3. Launches the background MPC thread at `mpcDesiredFrequency` Hz

### `update()` @ 500 Hz *(TODO — step 3)*
```
updateStateEstimation()
  ├─ read jointNames_[i]/position, /velocity, /effort
  ├─ read torso_imu/orientation, angular_velocity, linear_acceleration
  ├─ stateEstimate_->updateJointStates() / updateContact() / updateImu()
  ├─ stateEstimate_->update() → measuredRbdState_
  └─ rbdConversions_->computeCentroidalState() → currentObservation_.state

mpcMrtInterface_->setCurrentObservation(currentObservation_)
mpcMrtInterface_->updatePolicy()
mpcMrtInterface_->evaluatePolicy() → desiredState, desiredInput

wbc_->update() → jointTorques

safetyChecker_->check()

write jointNames_[i]/position, /velocity, /effort
```

### `on_deactivate()` *(TODO — step 3)*
Sets `controllerRunning_ = false`, joins the MPC thread.

---

## Update rates

There are three independent rates, each defined in a different place.

### 1. Physics simulation rate — `biped_description/mjcf/biped_robot_12dof.xml`
```xml
<option timestep="0.001"/>   <!-- 1000 Hz -->
```
The MuJoCo internal physics step (`mj_step()`). Controls simulation accuracy, not control frequency.
`mujoco_system_interface` checks that this is **≥** the controller rate and warns if violated.

### 2. Hardware read/write + controller update rate — `config/main_controllers.yaml`
```yaml
controller_manager:
  ros__parameters:
    update_rate: 500   # 500 Hz
```
This single value drives **three things simultaneously** in the ros2_control loop:
1. `hardware_interface::read()` — copies `mj_data_` sensor values into state interfaces (joint positions, IMU, …)
2. all controllers' `update()` — `LeggedController::update()` runs at this rate
3. `hardware_interface::write()` — applies command interfaces back to `mj_data_`

So `update_rate` is also the rate at which state (sensor) data is refreshed.

With `timestep = 0.001 s` (1000 Hz) and `update_rate = 500 Hz`, MuJoCo steps **twice** per controller
cycle — giving smoother dynamics while the controller still runs at 500 Hz.

### 3. MPC / MRT frequencies — `config/mpc/task.info`
```
mpcDesiredFrequency   80 Hz   ; background MPC solver thread
mrtDesiredFrequency  500 Hz   ; policy evaluation — must match update_rate
```
OCS2-level rates. `mrtDesiredFrequency` **must equal** `update_rate` — that is why the comment in
`main_controllers.yaml` says "must match mujoco_ros2_control_node update rate".

### Rate summary

| Rate | Value | Where defined | What it controls |
|---|---|---|---|
| Physics step | **1000 Hz** | `mjcf/biped_robot_12dof.xml` `<option timestep>` | MuJoCo `mj_step()` accuracy |
| Hardware read/write + controller update | **500 Hz** | `main_controllers.yaml` `update_rate` | `read()` → `update()` → `write()` cycle |
| MPC solver | **80 Hz** | `task.info` `mpcDesiredFrequency` | Background MPC thread |
| MRT policy evaluation | **500 Hz** | `task.info` `mrtDesiredFrequency` | Must equal `update_rate` |

---

## Config files

| File | Purpose |
|---|---|
| `config/main_controllers.yaml` | Controller Manager plugin registration + `joint_state_broadcaster` joint list + `imu_sensor_broadcaster` sensor name |
| `config/mpc/task.info` | Full MPC/OCS2 problem definition (solver, costs, constraints, Kalman filter) |
| `config/mpc/reference.info` | Nominal standing posture reference |
| `config/mpc/gait.info` | Gait schedule library (stance, trot, …) |
| `config/mujoco_interface_pd_gains.yaml` | PD gains used by the MuJoCo hardware plugin to hold position before the controller activates |
| `config/legged_hw_interface.yaml` | Hardware interface config for the real robot (`legged_hw` backend) |

---

## Launch files

| File | Purpose |
|---|---|
| `launch/test_load_mpc_controller.launch.py` | Full simulation stack: `robot_state_publisher` + MuJoCo + broadcasters + `legged_controller`. Use this to verify the complete lifecycle. |
| `launch/mujoco_sim.launch.py` | MuJoCo simulation without the legged controller (broadcasters only). |
| `launch/real_hw.launch.py` | Real hardware launch using the `legged_hw` backend. |

**Running the test launch:**
```bash
colcon build --packages-select legged_controllers
source install/setup.bash
ros2 launch legged_controllers test_load_mpc_controller.launch.py

# Optional arguments:
ros2 launch legged_controllers test_load_mpc_controller.launch.py headless:=true
ros2 launch legged_controllers test_load_mpc_controller.launch.py pd_gains_file:=/path/to/gains.yaml
```

---

## Dependencies

| Package | Role |
|---|---|
| `controller_interface` | `ControllerInterface` base class and lifecycle |
| `hardware_interface` | Command/state interface types |
| `pluginlib` | Runtime plugin discovery and loading |
| `rclcpp` / `rclcpp_lifecycle` | ROS 2 node and lifecycle node |
| `legged_interface` | OCS2 optimal control problem for legged robots |
| `legged_estimation` | `KalmanFilterEstimate`, `StateEstimateBase` |
| `ocs2_mpc` | `MPC_MRT_Interface`, `SystemObservation` |
| `ocs2_centroidal_model` | `CentroidalModelRbdConversions`, state vector definitions |
| `ocs2_core` | Benchmark timers, math utilities |

---

## Implementation status

| Component | Status |
|---|---|
| `on_init()` — parameter loading | Done |
| `command_interface_configuration()` | Done |
| `state_interface_configuration()` | Done |
| `on_configure()` — OCS2 setup | TODO (step 3) |
| `on_activate()` — start MPC thread | TODO (step 3) |
| `update()` — full control loop | TODO (step 3) |
| `on_deactivate()` — thread cleanup | TODO (step 3) |
| `LeggedCheaterController` | TODO (post-core) |
