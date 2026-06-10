# legged_hw

A ros2_control hardware interface plugin for the legged biped robot. It bridges
UDP state packets (from MuJoCo simulation or a real STM32-based robot) into the
ros2_control framework, and sends joint commands back via UDP.

## Architecture

```
MuJoCo / STM32 board
      │
      │  UDP  RobotStatePacket
      │  (joint pos/vel/torque + IMU)
      ▼
LeggedHW::recvLoop()          ← background thread, fills latest_state_ buffer
LeggedHW::watchdogLoop()      ← background thread, logs warning every 2 s while
      │                           no data has arrived yet
LeggedHW::read()              ← called every control cycle by controller_manager
      │  copies buffer → ros2_control state interfaces
      │
      ├──────────────────────────────────────┐
      ▼                                      ▼
joint_state_broadcaster             imu_sensor_broadcaster
reads joint pos / vel / effort      reads orientation / gyro / accel
publishes:                          publishes:
  /legged_hw/joint_states             /legged_hw/imu_sensor_broadcaster/imu
  (sensor_msgs/JointState)            (sensor_msgs/Imu)
      │                                      │
      └──────────────┬───────────────────────┘
                     ▼
          MPC / WBC / state estimator
                     │
          writes joint commands to
          ros2_control command interfaces
                     │
              LeggedHW::write()     ← packages commands into JointCommandPacket
                     │              ← skipped when all kp/kd == 0 (no active
                     │                motion controller)
                     │  UDP  JointCommandPacket
                     │  (pos_des / vel_des / torque_ff / kp / kd)
                     ▼
          MuJoCo / STM32 board
```

### Key roles

**`LeggedHW` (hardware plugin)**
- Loaded by `controller_manager/ros2_control_node` when it reads
  `<plugin>legged_hw/LeggedHW</plugin>` from the robot description.
- `on_activate()` opens UDP sockets and starts the receive and watchdog threads.
- `read()` snapshots the latest UDP packet into ros2_control state interface
  buffers every control cycle. Always returns `OK` (even before hardware
  connects) so the controller_manager update loop keeps running.
- `write()` packs ros2_control command interface buffers into a
  `JointCommandPacket` and sends it via UDP, but only when at least one joint
  has non-zero kp or kd — this prevents zero-gain packets from overriding the
  robot's internal hold mode before a motion controller is active.
- `watchdogLoop()` runs in a background thread and logs a warning every 2 s
  while no UDP data has been received yet, allowing ros2_control to activate
  even when the hardware is not yet connected.

**`joint_state_broadcaster`**
- A ros2_control *controller* (not a hardware interface).
- Reads joint `position`, `velocity`, `effort` state interfaces from `LeggedHW`
  and publishes them as `sensor_msgs/JointState` on `/legged_hw/joint_states`.

**`imu_sensor_broadcaster`**
- A ros2_control *controller* (not a hardware interface).
- Reads the `torso_imu` sensor state interfaces (`orientation`, `angular_velocity`,
  `linear_acceleration`) from `LeggedHW` and publishes them as `sensor_msgs/Imu`
  on `/legged_hw/imu_sensor_broadcaster/imu`.

Without the broadcasters, `LeggedHW` still receives and buffers UDP data — but
nothing publishes it to the ROS graph (RViz, state estimator, MPC, etc.).

## UDP Packet Format

Both structs must stay byte-identical between this package and
`mujoco_ros2_control` (or the real robot firmware). They are defined in
`include/legged_hw/udp_bridge.hpp`.

### `RobotStatePacket` — robot/sim → legged_hw (port 9870)

| Field | Type | Description |
|---|---|---|
| `timestamp_ns` | `uint64` | System clock timestamp (ns) |
| `num_joints` | `uint32` | Number of valid joint entries |
| `joint_pos[32]` | `float[32]` | Joint positions (rad) |
| `joint_vel[32]` | `float[32]` | Joint velocities (rad/s) |
| `joint_torque[32]` | `float[32]` | Joint torques (Nm) |
| `imu_quat[4]` | `float[4]` | IMU quaternion (w, x, y, z) |
| `imu_gyro[3]` | `float[3]` | Angular velocity (rad/s) |
| `imu_accel[3]` | `float[3]` | Linear acceleration (m/s²) |

### `JointCommandPacket` — legged_hw → robot/sim (port 9871)

| Field | Type | Description |
|---|---|---|
| `timestamp_ns` | `uint64` | System clock timestamp (ns) |
| `num_joints` | `uint32` | Number of valid joint entries |
| `pos_des[32]` | `float[32]` | Desired positions (rad) |
| `vel_des[32]` | `float[32]` | Desired velocities (rad/s) |
| `torque_ff[32]` | `float[32]` | Feedforward torques (Nm) |
| `kp[32]` | `float[32]` | Position gains (Nm/rad) |
| `kd[32]` | `float[32]` | Velocity gains (Nm·s/rad) |

## ros2_control Interfaces

### State interfaces (read by broadcasters / controllers)

| Interface | Description |
|---|---|
| `<joint>/position` | Joint position (rad) |
| `<joint>/velocity` | Joint velocity (rad/s) |
| `<joint>/effort` | Joint torque (Nm) |
| `torso_imu/orientation.{w,x,y,z}` | IMU quaternion |
| `torso_imu/angular_velocity.{x,y,z}` | IMU gyroscope (rad/s) |
| `torso_imu/linear_acceleration.{x,y,z}` | IMU accelerometer (m/s²) |

### Command interfaces (written by motion controllers)

| Interface | Description |
|---|---|
| `<joint>/position` | Desired joint position (rad) |
| `<joint>/velocity` | Desired joint velocity (rad/s) |
| `<joint>/effort` | Feedforward torque (Nm) |
| `<joint>/kp` | PD position gain (Nm/rad) |
| `<joint>/kd` | PD velocity gain (Nm·s/rad) |

## URDF Configuration

The hardware block is enabled with `use_legged_hw:=true` in
`biped_description/urdf/biped_robot_12dof.urdf`. The `use_sim` param selects
the target IP:

```xml
<ros2_control name="LeggedSystem" type="system">
  <hardware>
    <plugin>legged_hw/LeggedHW</plugin>
    <param name="use_sim">true</param>          <!-- true → sim_ip, false → hw_ip -->
    <param name="sim_ip">127.0.0.1</param>      <!-- MuJoCo on localhost -->
    <param name="hw_ip">192.168.1.100</param>   <!-- STM32 board IP -->
    <param name="state_port">9870</param>        <!-- UDP port to receive state on -->
    <param name="cmd_port">9871</param>          <!-- UDP port to send commands to -->
  </hardware>
  <!-- 12 joints + torso_imu sensor ... -->
</ros2_control>
```

The STM32 board must send `RobotStatePacket` UDP datagrams to **this machine's
IP** on port **9870**. Commands are sent back to `hw_ip:9871`.

## Launch

### MuJoCo simulation with legged_hw bridge (default)

Starts MuJoCo (UDP enabled) alongside the legged_hw ros2_control node.

```bash
ros2 launch biped_centroidal_mpc mujoco_sim.launch.py
```

### MuJoCo simulation only (no legged_hw)

```bash
ros2 launch biped_centroidal_mpc mujoco_sim.launch.py use_legged_hw:=false
```

### Real hardware (STM32 board, no MuJoCo)

```bash
ros2 launch biped_centroidal_mpc real_hw.launch.py
```

Controllers activate immediately. If the board is not yet connected, you will
see a warning every 2 s:

```
[LeggedHW]: No UDP data from hardware yet — waiting for 192.168.1.100:9870 ...
```

Once the board starts sending packets the warning stops and normal operation
begins:

```
[LeggedHW]: Hardware connected — receiving UDP data.
```

Verify data is flowing:

```bash
ros2 topic echo /legged_hw/joint_states
ros2 topic echo /legged_hw/imu_sensor_broadcaster/imu
```

### mujoco_sim.launch.py arguments

| Argument | Default | Description |
|---|---|---|
| `use_legged_hw` | `true` | Enable legged_hw node alongside MuJoCo |
| `legged_hw_use_sim` | `true` | `true` → connect to MuJoCo (`sim_ip`); `false` → STM32 (`hw_ip`) |
| `legged_hw_sim_ip` | `127.0.0.1` | IP of the MuJoCo machine |
| `legged_hw_hw_ip` | `192.168.1.100` | IP of the STM32 board |
| `legged_hw_state_port` | `9870` | UDP port legged_hw listens on for state |
| `legged_hw_cmd_port` | `9871` | UDP port legged_hw sends commands to |
| `headless` | `false` | Run MuJoCo without a GUI window |

## Package Structure

```
legged_hw/
├── include/legged_hw/
│   ├── legged_hw.hpp       # Hardware interface class declaration
│   └── udp_bridge.hpp      # UDP packet structs + socket helper + IP constants
├── src/
│   └── legged_hw.cpp       # Implementation (recvLoop, watchdogLoop, read, write)
├── CMakeLists.txt
├── package.xml
├── legged_hw_plugin.xml    # pluginlib export
└── README.md
```

## Dependencies

- `hardware_interface` (ros2_control)
- `pluginlib`
- `rclcpp`, `rclcpp_lifecycle`
