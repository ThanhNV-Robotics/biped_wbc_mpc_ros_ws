# What happens when you run `sim_floating.launch.py`

```bash
ros2 launch biped_bringup sim_floating.launch.py            # MuJoCo viewer + RViz
ros2 launch biped_bringup sim_floating.launch.py headless:=true rviz:=false
```

Result: the v2 biped stands on the MuJoCo floor at the `home` pose, held by the simulated motors
(fixed kp / kd PD) with `position_controller` sending the home joint angles.

This page follows the launch in the order things happen. Everything below was checked on a real
run (2026-10-23, ROS 2 Humble, ros2_control 2.54).

---

## 0. Overview

```
ros2 launch
 │
 ├─ (load time) xacro biped_hw.urdf.xacro ──► URDF string = robot_description
 │
 ├─ robot_state_publisher ◄── robot_description, /joint_states ──► /tf, /robot_description
 │
 ├─ ros2_control_node (controller_manager) ◄── robot_description, controllers.yaml,
 │   │                                         controllers_sim_floating.yaml
 │   └─ BipedSystem = MujocoSystemInterface (plugin) ── loads scene_floatingbase_12dof_v2.xml
 │       ├─ MuJoCo physics thread (1 kHz) + viewer ──► /clock, /simulator/floating_base_state
 │       └─ hardware ACTIVE: motors hold the keyframe pose
 │
 ├─ spawner joint_state_broadcaster ─► loads + activates it, exits
 │        └─(on exit)─ spawner position_controller ─► loads + activates it, exits
 │                          └─(on exit)─ ros2 topic pub --once HOME ─► exits
 │
 ├─ odom_to_tf.py ◄── /simulator/floating_base_state ──► /tf (world → link0_torso)
 │
 └─ rviz2 (if rviz:=true)
```

---

## 1. Launch arguments

Declared at the bottom of the file with `DeclareLaunchArgument`, read with
`LaunchConfiguration('name')`, set on the command line with `name:=value`
(`ros2 launch biped_bringup sim_floating.launch.py --show-args` lists them).

| Argument | Default | Goes to |
|---|---|---|
| `model` | `v2_biped_robot_12dof.urdf` | xacro: which URDF to include |
| `gains` | `<share>/biped_hardware/config/joint_gains_sim.yaml` | xacro: kp / kd of every joint |
| `mujoco_scene` | `scene_floatingbase_12dof_v2.xml` | xacro → hardware param `mujoco_model` |
| `initial_keyframe` | `home` | xacro → hardware param `initial_keyframe` |
| `headless` | `false` | xacro → hardware param `headless` (no MuJoCo window) |
| `rviz` | `true` | launch only: start rviz2 or not |

---

## 2. Building `robot_description` (before any node starts)

The launch runs `xacro` once and keeps its output (a complete URDF, as one string):

```bash
xacro <share>/biped_hardware/urdf/biped_hw.urdf.xacro \
  use_sim:=true model:=v2_biped_robot_12dof.urdf \
  gains:=<share>/biped_hardware/config/joint_gains_sim.yaml \
  mujoco_scene:=scene_floatingbase_12dof_v2.xml initial_keyframe:=home headless:=false
```

`ParameterValue(..., value_type=str)` keeps the XML as a plain string (not parsed as YAML).

Files read by xacro:

| File | Package | Content used |
|---|---|---|
| `urdf/biped_hw.urdf.xacro` | biped_hardware | top file: arguments, the `<ros2_control>` block |
| `urdf/v2_biped_robot_12dof.urdf` | biped_description | `<xacro:include>`: 14 links / joints, inertias, meshes (`package://biped_description/urdf/meshes/*.stl`) |
| `config/joint_gains_sim.yaml` | biped_hardware | `xacro.load_yaml`: kp / kd per joint |

What the `<ros2_control name="BipedSystem">` block becomes with `use_sim:=true`:

- `<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>` (instead of the real
  `biped_hardware/BipedSystem`)
- hardware params: `mujoco_model` (full path of the scene), `headless`, `initial_keyframe`,
  `pvt_mode=true`, `command_timeout=0.02`, `damping_kd=1.0`, `damping_duration=1.0`
- 12 `<joint>` entries, each with params `kp`, `kd` (`motor_index` / `direction` are ignored in
  sim), command interfaces `position`, `velocity`, `effort`, state interfaces `position`,
  `velocity`, `effort`

---

## 3. Processes started at once

| # | Process (package / executable) | Node name | Parameters |
|---|---|---|---|
| 1 | `robot_state_publisher` / `robot_state_publisher` | `/robot_state_publisher` | `robot_description`, `use_sim_time` |
| 2 | `mujoco_ros2_control` / `ros2_control_node` | `/controller_manager` (+ `/mujoco_clock` inside the same process) | `robot_description`, `controllers.yaml`, `controllers_sim_floating.yaml`, `use_sim_time` |
| 3 | `controller_manager` / `spawner` | `/spawner_joint_state_broadcaster` | args `joint_state_broadcaster --controller-manager /controller_manager` |
| 4 | `biped_bringup` / `odom_to_tf.py` | `/odom_to_tf` | `use_sim_time` |
| 5 | `rviz2` / `rviz2` (if `rviz:=true`) | `/rviz2` | `-d <share>/biped_description/rviz/display.rviz` |

`use_sim_time: true` everywhere: time comes from `/clock` (MuJoCo sim time), not the PC clock.
So every node also subscribes to `/clock`; it is left out of the lists below.

### 3.1 `robot_state_publisher`
**What for:** turns joint angles into link poses, so RViz (and any TF user) knows where every link
of the robot is, relative to the torso.

- **Subscribes:** `/joint_states` (`sensor_msgs/JointState`, from `joint_state_broadcaster`)
- **Publishes:**
  - `/tf`: one transform per moving joint (e.g. `link0_torso → left_hip_pitch_link`), computed
    from the URDF joint origin + the joint angle, each time a `/joint_states` message arrives
  - `/tf_static`: transforms of fixed joints (once, latched)
  - `/robot_description` (`std_msgs/String`): the URDF it was given, latched, for RViz
- **Does not** know where the torso is in the world: that is `odom_to_tf.py` (3.4).

### 3.2 `ros2_control_node` (controller_manager + MuJoCo)
**What for:** the core of the simulation. One process that contains
- the **controller_manager**: loads / starts / stops controllers and runs them in a 500 Hz loop
- the **hardware plugin** `MujocoSystemInterface` (named `BipedSystem`): the MuJoCo simulation
  and the emulated motor drives, seen by the controllers through hardware interfaces
- later, the **controllers** themselves (`joint_state_broadcaster`, `position_controller`), as
  plugins inside the same process

- **Subscribes:** nothing at startup. The controllers add their own subscriptions when they are
  loaded (`position_controller` → `/position_controller/commands`).
- **Publishes:**
  - `/clock` (`rosgraph_msgs/Clock`), from the internal node `/mujoco_clock`: the MuJoCo sim
    time, after every physics step
  - `/simulator/floating_base_state` (`nav_msgs/Odometry`): torso pose and velocity read from
    the free joint (frame `odom`, child `link0_torso`), ground truth
  - the controllers' topics once they are active (`/joint_states`, `/dynamic_joint_states`,
    `/<controller>/transition_event`)
- **Services:** `/controller_manager/*` (load / configure / switch / list ..., see 4.4)
- **Details:** section 4

### 3.3 `spawner joint_state_broadcaster`
**What for:** a short-lived helper (a command-line tool run as a node): it asks the
controller_manager to load, configure and activate `joint_state_broadcaster`, then exits.

- **Subscribes / publishes:** no topics
- **Uses services:** `/controller_manager/list_controllers`, `load_controller`,
  `configure_controller`, `switch_controller` (as a client)
- **Exit** is the trigger for the next step (section 5). The broadcaster keeps running inside the
  controller_manager after the spawner is gone.

### 3.4 `odom_to_tf.py`
**What for:** places the robot in the world for RViz. The simulator publishes the torso pose as
odometry, but RViz needs it as TF.

- **Subscribes:** `/simulator/floating_base_state` (`nav_msgs/Odometry`)
- **Publishes:** `/tf`: `world → link0_torso` (same stamp, position and orientation as the
  message; parent renamed from `odom` to `world`, the RViz fixed frame)
- **Parameters:** `parent_frame` (default `world`), `odom_topic` (default
  `/simulator/floating_base_state`)
- Ground truth, sim only: on the robot this transform has to come from the state estimator.

Together with 3.1 this gives the full TF tree:
`world → link0_torso` (odom_to_tf) `→ hips → knees → ankles` (robot_state_publisher).

### 3.5 `rviz2` (only if `rviz:=true`)
**What for:** 3D view of the robot. Config `biped_description/rviz/display.rviz`, fixed frame
`world`, displays Grid, RobotModel and TF.

- **Subscribes:**
  - `/robot_description`: RobotModel display (link meshes, `package://biped_description/...`)
  - `/tf`, `/tf_static`: where to draw each link
- **Publishes:** nothing used by the simulation (only its own interactive tool topics, e.g.
  `/clicked_point`, `/goal_pose`, `/initialpose`)
- Only a viewer: closing it does not affect the simulation. The MuJoCo window (`headless:=false`)
  is a separate viewer, inside `ros2_control_node`.

---

## 4. Inside `ros2_control_node` (the important part)

`ros2_control_node` here is the executable of the vendored `mujoco_ros2_control` package (not the
stock `controller_manager/ros2_control_node`). It creates a `controller_manager`, which:

### 4.1 Reads its parameters
- `config/controllers.yaml`: `update_rate: 500`, the controller names and types
  (`joint_state_broadcaster`, `joint_jogging_controller`, `position_controller`), their
  parameters, and `hardware_components_initial_state.inactive: [BipedSystem]`.
- `config/controllers_sim_floating.yaml`, loaded **after** it, overrides that list with
  `inactive: [none]` → `BipedSystem` is not kept inactive, it gets activated (log:
  `Hardware component 'none' is unknown ...`, expected).

### 4.2 Loads the hardware plugin (`libmujoco_ros2_control.so`) and runs its lifecycle
1. **init** (`on_init`)
   - loads the MJCF scene `biped_description/mjcf/scene_floatingbase_12dof_v2.xml`, which
     `<include>`s `v2_biped_robot_floatingbase_12dof.xml` (robot: free joint
     `floating_base_joint`, 12 hinge joints, 12 `<motor>` actuators, foot contact boxes, meshes
     from `mjcf/meshes/`), plus floor, lights and a step box
   - starts the MuJoCo viewer thread (unless `headless`) and the **physics thread**
     (timestep 1 ms) – from here gravity acts
   - applies keyframe `home`: torso at z = 0.848 m, legs `[-0.2, 0, 0, 0.4, 0, -0.2]` each
   - creates node `/mujoco_clock` publishing `/clock`, and the publisher
     `/simulator/floating_base_state` (free joint → `nav_msgs/Odometry`)
   - PVT drive: reads kp / kd per joint, starts with the motors DISABLED
2. **configure**
3. **activate** (because of 4.1): range check, then every motor holds its current angle
   (= keyframe) with its kp / kd. Log: `ACTIVE: 12 simulated motors enabled ...`

### 4.3 Starts the control loop (500 Hz, sim time)
Every cycle: `read()` (joint states from MuJoCo) → `update()` of all active controllers →
`write()` (joint commands to the drive). The drive applies, before each 1 kHz physics step:

```
tau = kp * (p_cmd - q) + kd * (v_cmd - dq) + tau_ff
```

If `write()` stops for > 20 ms (sim time): damping (kd 1) for 1 s, then motors disabled – like the
real firmware.

### 4.4 Offers its services
`/controller_manager/load_controller`, `configure_controller`, `switch_controller`,
`list_controllers`, `list_hardware_components`, `list_hardware_interfaces`,
`set_hardware_component_state`, `unload_controller`, ... – used by the spawners and by
`ros2 control ...`.

---

## 5. The chained steps

Started one after the other with `RegisterEventHandler(OnProcessExit(...))`: each starts when the
previous process exits.

### 5.1 `spawner joint_state_broadcaster` (started in step 3)
Waits for `/controller_manager`, then load → configure → activate, then exits.
- type `joint_state_broadcaster/JointStateBroadcaster` (from `/opt/ros/humble`)
- reads the 12 × (position, velocity, effort) state interfaces, claims **no** command interface
- creates node `/joint_state_broadcaster`, publishes `/joint_states` and `/dynamic_joint_states`

### 5.2 `spawner position_controller` (when 5.1 exits)
- type `forward_command_controller/ForwardCommandController` (from `/opt/ros/humble`)
- parameters from `controllers.yaml`: 12 joints, `interface_name: position`
- claims the 12 **position** command interfaces (velocity / effort stay unclaimed, stay 0)
- creates node `/position_controller`, subscribes `/position_controller/commands`
  (`std_msgs/Float64MultiArray`)
- writes nothing until the first message → motors keep the activation pose

### 5.3 `ros2 topic pub` (when 5.2 exits)
```bash
ros2 topic pub --once -w 1 /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [-0.2, 0.0, 0.0, 0.4, 0.0, -0.2, -0.2, 0.0, 0.0, 0.4, 0.0, -0.2]}"
```
`-w 1` waits for the controller's subscriber, `--once` sends one message and exits. `HOME` is
defined at the top of the launch file (same values as the keyframe). From now on the controller
writes HOME to the position commands every cycle.

---

## 6. Steady state

Nodes (`ros2 node list`):

| Node | From |
|---|---|
| `/robot_state_publisher` | robot_state_publisher |
| `/controller_manager` | ros2_control_node |
| `/mujoco_clock` | ros2_control_node (MuJoCo hardware plugin) |
| `/joint_state_broadcaster` | controller inside ros2_control_node |
| `/position_controller` | controller inside ros2_control_node |
| `/odom_to_tf` | odom_to_tf.py |
| `/rviz2` | rviz2 (if started) |

The spawners and `ros2 topic pub` are gone (they exit after their job).

Topics (`ros2 topic list -t`):

| Topic | Type | Publisher → subscriber |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | `/mujoco_clock` → every node (sim time) |
| `/joint_states` | `sensor_msgs/JointState` | `/joint_state_broadcaster` → `/robot_state_publisher` |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | `/joint_state_broadcaster` (incl. `BipedSystem/enabled`) |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | you / `ros2 topic pub` → `/position_controller` |
| `/simulator/floating_base_state` | `nav_msgs/Odometry` | MuJoCo plugin → `/odom_to_tf` (ground-truth torso pose) |
| `/tf` | `tf2_msgs/TFMessage` | `/robot_state_publisher` (links), `/odom_to_tf` (`world → link0_torso`) → rviz2 |
| `/tf_static` | `tf2_msgs/TFMessage` | `/robot_state_publisher` (fixed joints) |
| `/robot_description` | `std_msgs/String` | `/robot_state_publisher` (the URDF, latched) |
| `/<controller>/transition_event` | `lifecycle_msgs/TransitionEvent` | controller lifecycle changes |

Controller ↔ hardware data does **not** go over topics: controllers run inside the
controller_manager process and use the hardware interfaces directly (this is why rqt_graph does not
show it). Check it with:

```bash
ros2 control list_controllers -v          # who claims what
ros2 control list_hardware_interfaces     # [claimed] / [unclaimed]
ros2 control list_hardware_components     # BipedSystem: active
```

---

## 7. All files involved

| File | Package | Used by |
|---|---|---|
| `launch/sim_floating.launch.py` | biped_bringup | `ros2 launch` |
| `urdf/biped_hw.urdf.xacro` | biped_hardware | xacro (step 2) |
| `urdf/v2_biped_robot_12dof.urdf` + `urdf/meshes/*.stl` | biped_description | xacro, robot_state_publisher, rviz2 |
| `config/joint_gains_sim.yaml` | biped_hardware | xacro → kp / kd in the URDF |
| `config/controllers.yaml` | biped_bringup | controller_manager + controllers |
| `config/controllers_sim_floating.yaml` | biped_bringup | controller_manager (activate at startup) |
| `mjcf/scene_floatingbase_12dof_v2.xml` (keyframe `home`) | biped_description | MuJoCo plugin |
| `mjcf/v2_biped_robot_floatingbase_12dof.xml` + `mjcf/meshes/*.stl` | biped_description | included by the scene |
| `scripts/odom_to_tf.py` | biped_bringup | node `/odom_to_tf` |
| `rviz/display.rviz` | biped_description | rviz2 (fixed frame `world`) |
| `lib/mujoco_ros2_control/ros2_control_node`, `lib/libmujoco_ros2_control.so` | mujoco_ros2_control | controller_manager executable, hardware plugin |
| `joint_state_broadcaster`, `forward_command_controller` | /opt/ros/humble | controller plugins |

`<share>` = `install/<package>/share/<package>`. Launch, config, URDF and MJCF files are read from
`install/`, not `src/`: rebuild after editing them, or build with
`colcon build --symlink-install` so `install/` links to `src/`.

---

## 8. Things that must stay consistent

- `HOME` in the launch file and keyframe `home` in the scene: same joint values (otherwise the
  robot starts at one pose and jumps to the other).
- Joint order of `HOME` = `joints:` of `position_controller` in `controllers.yaml`
  (left hip pitch, hip roll, hip yaw, knee, ankle roll, ankle pitch, then right).
- Joint names: URDF, MJCF, `joint_gains_sim.yaml`, `controllers.yaml`.
- Only one controller may claim the position interfaces: deactivate `position_controller` before
  activating `joint_jogging_controller` or a WBC that uses them
  (`ros2 control switch_controllers --deactivate position_controller --activate <other>`).
