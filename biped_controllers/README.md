# biped_controllers

ros2_control controllers for the 12-motor biped. They run inside the `controller_manager` loop
(500 Hz) together with the hardware interface `biped_hardware/BipedSystem`.

| Controller | Plugin | Use |
|---|---|---|
| `joint_jogging_controller` | `biped_controllers/JointJoggingController` | Move joints with `joint_state_publisher_gui` sliders, slowly and smoothly. Test tool for the hardware interface. |

The whole-body controller will go into this package later.

## joint_jogging_controller

```bash
ros2 launch biped_bringup hardware.launch.py slider:=true
```

That launch activates the hardware, spawns the controller and opens the sliders (remapped to
`/slider_joint_states`, because `/joint_states` is the real robot).

**Interfaces**

| | Per joint | Extra |
|---|---|---|
| Command (claimed) | `position`, `velocity`, `effort` | – |
| State | `position` | `BipedSystem/enabled` |

**Each cycle (`update()`)**, per joint:

1. goal = slider value if that slider has been moved since activation, else the pose it started at
2. rate limit: the target moves at most `max_velocity · dt` toward the goal
3. first-order low-pass with `filter_time_constant`
4. writes position = filtered target, velocity = 0, effort = 0

The motors track the position with their own kp / kd (the hardware sends PVT frames only).

**Safety**

- On activation every target starts at the measured position.
- A joint follows its slider only after that slider has been moved (the GUI starts all sliders
  at 0, untouched joints keep their pose).
- While `BipedSystem/enabled` is 0 (motors disabled), the target follows the measured position.
  Humble lets the controller be active while the hardware is inactive; without this, moving a leg
  by hand and then enabling the motors would make them jump back to the old pose.
- The subscription runs in a normal thread and hands the slider values to `update()` through a
  `realtime_tools::RealtimeBuffer`: no locks and no allocation in the real-time loop.

**Parameters** (`biped_bringup/config/controllers.yaml`)

| Name | Default | Meaning |
|---|---|---|
| `joints` | – (required) | joints to command |
| `slider_topic` | `/slider_joint_states` | `sensor_msgs/JointState` with the slider values |
| `max_velocity` | 0.3 rad/s | how fast a joint moves toward its slider |
| `filter_time_constant` | 0.1 s | low-pass after the rate limit (0 = off) |
| `enabled_interface` | `BipedSystem/enabled` | state interface that is 1 while the motors are enabled (empty = not used) |
