# biped_hardware

ros2_control hardware interface (`hardware_interface::SystemInterface`) for the 12-motor biped.
The PC talks over Ethernet UDP to a **NUCLEO-H755ZI-Q** board, which drives the 12 MAB MD80 motor
drivers over two CAN FD buses.

> **Current stage: read and write path done.** The hardware starts **inactive** (states only,
> motors disabled). Activating it writes the kp / kd from `config/joint_gains.yaml` into the
> motors (verified by read-back), enables them, and they **hold the current pose with those gains**.
> While active the gains are fixed: PVT commands only. Status and next steps: [NOTES.md](NOTES.md).

---

## System overview

```
 PC (ROS 2 Humble)                                   NUCLEO-H755ZI-Q                       Motors
 ┌──────────────────────────────────┐   UDP :9999    ┌──────────────┐  SRAM4   ┌──────────┐  FDCAN1  ┌───────────┐
 │ controller_manager (500 Hz)      │  cmd 256 B ──▶ │ CM7          │ mailbox  │ CM4      │ ───────▶ │ left leg  │
 │   read() ─ update() ─ write()    │                │ LwIP, UDP    │ ───────▶ │ 500 Hz   │          │ IDs 10-60 │
 │ BipedSystem (this package)       │ ◀── state 224 B│ server       │ ◀─────── │ CAN loop │  FDCAN2  ┌───────────┐
 └──────────────────────────────────┘                └──────────────┘          └──────────┘ ───────▶ │ right leg │
   192.168.10.1                                        192.168.10.10                                 │ IDs 70-120│
                                                                                                     └───────────┘
```

- **PC:** `controller_manager` runs `read()`, the controllers' `update()` and `write()` at 500 Hz.
- **CM7:** answers each command packet with one state packet.
- **CM4:** runs the CAN cycle at a fixed 500 Hz and applies the latest command. A slow or missing PC
  never stalls the motor loop. (1 kHz was tried: no room for resending lost replies, see
  [Timing](#timing-pc--motors--pc).)

The firmware lives in `../../STM32H755_WS_FDCan_UDP` (its own git repo, see its `README.md` and
`FDCanUdp/NOTES.md`).

---

## Package layout

```
biped_hardware/
├── include/biped_hardware/
│   ├── biped_system.hpp       BipedSystem: the ros2_control plugin
│   ├── udp_client.hpp         non-blocking UDP socket connected to the board
│   ├── wbc_protocol.h         packet layout, byte-identical copy of the firmware header
│   └── ankle_kinematics.hpp   parallel ankle: motor pair <-> roll / pitch
├── src/
│   ├── biped_system.cpp
│   └── udp_client.cpp
├── urdf/biped_hw.urdf.xacro   robot model from biped_description + <ros2_control> block
│                              (use_sim:=true: MuJoCo instead of BipedSystem)
├── config/joint_gains.yaml    kp / kd of the robot (section `robot`), also used by MuJoCo
└── biped_hardware.xml         pluginlib description (biped_hardware/BipedSystem)
```

Launch files and controller configuration are in **`biped_bringup`**:
`launch/hardware.launch.py` (real robot), `launch/sim.launch.py` (the same in MuJoCo,
mujoco_ros2_control in PVT mode), `launch/sim_floating.launch.py`, `launch/tsid_wbc.launch.py`,
`config/controllers.yaml` (controller_manager rate and controllers, robot and sim).

---

## UDP protocol (`wbc_protocol.h`)

PC → `192.168.10.10:9999`, one command per control cycle. The board replies to the sender with one
state packet. Little-endian, packed structs, sent as raw bytes.

| Packet | Size | Header | Per motor (× 12) |
|---|---|---|---|
| `WbcCommandPacket` | 256 B | `magic "WBC1"`, `version`, `flags` (`ENABLE`, `GAINS_VALID`), `seq`, `pcTimeUs` | `pos`, `vel`, `tau`, `kp`, `kd` (f32) |
| `WbcStatePacket` | 224 B | `magic "WST1"`, `version`, `status` (`ENABLED`, `CMD_TIMEOUT`, `MOTOR_FAULT`, `NO_CM4_DATA`), `seqEcho`, `stmTimeUs`, `cm4Cycle`, `cycleUs`, `cmdAgeUs` | `pos`, `vel`, `tau` (f32), `quickStatus`, `temperature`, `flags` (`ONLINE`, `ENABLED`, `MISSED`) |

Motor torque on the drive: `τ = kp·(pos − q) + kd·(vel − q̇) + tau`.

**Motor order** (`motors[]` index = CAN ID order):

| Index | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| CAN ID | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 | 110 | 120 |
| Motor | L hip pitch | L hip roll | L hip yaw | L knee | L ankle upper | L ankle lower | R hip pitch | R hip roll | R hip yaw | R knee | R ankle upper | R ankle lower |

**The header exists twice:** `FDCanUdp/Common/Inc/wbc_protocol.h` (firmware, the original) and
`include/biped_hardware/wbc_protocol.h`. Change both together and check with `cmp`. A mismatch
does not give an error, only wrong values.

---

## Design

### Control loop: synchronous, no threads

```
controller_manager cycle k (2 ms at 500 Hz):
  read()   drain the socket, keep the newest state  (= reply to write() of cycle k-1)
  update() controllers
  write()  one WbcCommandPacket, seq++
```

The board's round trip is about 0.23 ms, well inside one cycle. So there is no receive thread, no
mutex and no shared buffer. The socket is `connect()`ed to the board, so the kernel drops datagrams
from anyone else, and it is non-blocking, so `read()` and `write()` never wait on the network.

### Timing: PC → motors → PC

Three loops run on their own clocks and pass data through "latest value" mailboxes. None of them
waits for another, so a slow PC can't stall the motors and a slow CAN cycle can't stall the PC.

| Loop | Where | Rate | Does |
|---|---|---|---|
| `controller_manager` | PC | 500 Hz | `read()` → controllers → `write()`: one UDP command per cycle |
| UDP server | CM7 | per packet | puts the command in the SRAM4 command box, answers **at once** with the newest state from the state box |
| CAN cycle | CM4 | 500 Hz (2 ms) | takes the newest command, sends it to all 12 motors, collects the 12 replies, writes the state box |

```
time ─────────────────────────────────────────────────────────────────────────────────────▶

PC        write() k ─UDP~0.1ms─▶                          read() k+1 (reply to k, already waiting)
                                │                          ▲
CM7                             ├─▶ command box            │ reply = state box as it is NOW
                                └──── reply ───UDP~0.1ms───┘ (finished by the last CM4 cycle)

CM4       ┌ cycle n ─────────────┐     ┌ cycle n+1 ──────────────────────┐
          │ CAN update ~0.76 ms  │ idle│ takes command k                 │ idle
          └──────────────────────┘     │ motor 10 / 70 first (~0.05 ms)  │
                                       │ ankle 60 / 120 last  (~0.7 ms)  │
                                       └─ state box updated ─────────────┘
          ◀─────── 2 ms ──────────────▶
```

| Piece | Time | Source |
|---|---|---|
| UDP round trip PC → CM7 → PC | ~0.23 ms avg, p99 ~0.33 ms | measured (2026-10-19, `wbc_udp_test.py`) |
| CAN update of all 12 motors (`update avg` in the board log) | ~0.76 ms avg, worst ~1.4 ms with resends, window 1.8 ms | measured (2026-10-23, motors enabled, PVT frames) |
| Command waits in the box for the next CM4 cycle | 0–2 ms, ~1 ms on average | estimate: PC and CM4 both run at 500 Hz, not synchronised |
| Command reaches a motor after the cycle starts | ~0.05 ms (first in the queue) … ~0.7 ms (ankles, last) | estimate from the update timing |
| **Command: PC `write()` → motor** | **~0.2 … 2.8 ms** | sum of the rows above |
| **State: motor → PC `read()`** | **~1 … 3 ms old** | state of the last finished CM4 cycle + one PC cycle |

So: **about one 2 ms control cycle of delay each way.** That's normal for this design and fine for
a whole-body controller at 500 Hz. It matters for high-gain or high-bandwidth control (derivative
terms see a slightly old velocity), not for holding poses or walking at moderate speed.

**Why 500 Hz and not 1 kHz on CM4:** a PVT update of enabled motors takes ~0.76 ms. An MD80 does
not resend a reply that loses CAN arbitration, so the board resends the command after 0.3 ms of
silence, and that does not fit in the rest of a 1 ms cycle: 1.3–3.5 % of cycles lost a reply at
1 kHz, 1 in 30 000 at 500 Hz. The PC sends at 500 Hz anyway, so 1 kHz only sent each command twice.

**See it live** in every state packet (`WbcStatePacket`):

| Field | Meaning |
|---|---|
| `seqEcho` | `seq` of the command CM4 used in the cycle that produced this state. CM7 replies the moment a command arrives, before CM4 can use it, so `seq − seqEcho` is usually 1 (2 if CM4's cycle fell just before the previous command) |
| `cmdAgeUs` | how long that command had been waiting when CM4 took it (0–2 ms) |
| `cycleUs` | duration of that CAN update |
| `cm4Cycle` | CM4 cycle counter (CM7 marks the data stale if it stops moving for 20 ms) |
| motor `flags` `MISSED` | this motor's reply did not arrive in that cycle; its values are from the cycle before |

**Possible later improvement:** start the CM4 cycle when a command arrives instead of on its own
2 ms clock (with the clock as a fallback if the PC stalls). That removes the 0–2 ms wait and brings
PC → motors → PC down to roughly 0.3 ms (UDP) + 0.8 ms (CAN) ≈ 1.1 ms.

### Joint space ↔ motor space

ros2_control only sees the 12 URDF joints. `BipedSystem` converts states in `read()` and commands
in `write()`.

- **Hips and knees:** `joint = direction · motor` for position, velocity and torque (both ways).
- **Parallel ankles:** each ankle has two motors that move roll and pitch together.
  - The `*_ankle_roll_joint` carries the upper motor (50 / 110) and the `*_ankle_pitch_joint`
    carries the lower motor (60 / 120), same convention as the earlier `humanoid_hardware`.
  - `direction` is applied to each motor first, then `ankle_kinematics.hpp` maps the pair:
    - position: `left/rightJointAngles` (forward map, Newton iteration, clamped to roll ±15°,
      pitch ±30°)
    - velocity: `footVelFromMotor` (inverse of the motor Jacobian)
    - torque: `ankleTorqueFromMotor` (Jacobian transpose)
  - Commands, the other way: position `left/rightMotorAngles` (closed-form inverse), velocity
    `motorVelFromFoot` (both at the commanded pose), torque `motorTorqueFromAnkle` (Jacobian at the
    measured pose). `direction` is applied to each motor after the mapping.
  - **Range:** mechanical design roll ±15°, pitch ±30° (= URDF limits). With the geometry in
    `ankle_kinematics.hpp` the linkage does **not** reach the corners of that box:

    | pitch | -30° | -25° | -20° | -15° … +20° | +25° | +30° |
    |---|---|---|---|---|---|---|
    | reachable roll (left leg; right mirrored) | -1.8 … +0.1° | -12 … +10° | full ±15° | full ±15° | full ±15° | -9 … +7° |

    Full ±15° roll only for pitch -22.7° … +26.9°. Commanded poses go through
    `clampToWorkspace` first: clamp to the box, then keep the pitch and reduce the roll until the
    linkage reaches it, so the inverse map never returns NaN (tested on 64 000 poses, reachable
    poses unchanged, round trip 5e-15). If the real ankle reaches more than this, the geometry
    constants (`LEFT` / `RIGHT`) need checking.
  - Gains: the kp / kd configured for an ankle joint are those of the motor it carries (motor
    space); roll and pitch stiffness follow from the ankle Jacobian.
  - Both ankle motors read 0 when the ankle is level, so no offset is needed (checked:
    round trip error 3e-15 over ±0.3 rad, Jacobian matches finite differences for both legs).

Per-joint parameters in the URDF `<ros2_control>` block:

| Parameter | Meaning |
|---|---|
| `motor_index` | index into `motors[]` (0–11), each used once |
| `direction` | `1` or `-1`, so a positive motor angle matches the positive URDF joint angle (verified on the robot 2026-10-23) |

The start range for activation is each joint's URDF `<limit lower upper>`, read from the robot
description the hardware gets (`HardwareInfo::original_xml`).

Hardware parameters: `board_ip` (`192.168.10.10`), `board_port` (`9999`), `timeout_cycles` (`10`).

### Interfaces

| Per joint | Interfaces |
|---|---|
| State | `position`, `velocity`, `effort`; plus `BipedSystem/enabled` (1 while the motors are enabled, for controllers) |
| Command | `position`, `velocity`, `effort` |

### Gains: `config/joint_gains.yaml`

kp / kd per joint (ankles: the motor each ankle joint carries). The xacro loads the file
(launch argument `gains:=<file>`) and puts the values into the joint parameters; `on_init` checks
them (all 12 present, kp in [0, 1000], kd in [0, 50]) and `on_configure` shows them in the motor
status table, with a warning for joints with kp = 0 (limp when enabled).

On **activation** the enable command carries the gains with `GAINS_VALID`. The firmware then, with
the motors still disabled, writes targets (current pose), impedance mode and kp / kd to each motor,
**reads kp / kd back** and refuses to enable on any mismatch; then it enables all 12 back to back.
Writing at activation (not at configure) guarantees the gains are in the motors even if their power
was cycled in between (the gains live in the motors' RAM).

**While active the gains are fixed:** `write()` sends only position, velocity and feed-forward
torque (20-byte PVT frames on CAN); the firmware ignores gains in that state (`WBC_WRITE_GAINS 0`
in `wbc_loop.c`). Drive law: `τ = kp·(pos − q) + kd·(vel − q̇) + tau_ff`.

Verified on the robot 2026-10-23 with a test file (kp 0, kd 0.05): board log
`[WBC] gains L (kp/kd): 10:0.0/0.05 ...`, then `nucleotool md -i <id> info` showed kd 0.0500.

### Lifecycle and safety

| Callback | Does |
|---|---|
The hardware starts **inactive** (`hardware_components_initial_state.inactive` in
`biped_bringup/config/controllers.yaml`), so launching ROS never enables a motor.

| Callback | Does |
|---|---|
| `on_init` | Parse parameters. Require exactly 12 joints, unique `motor_index`, `direction` ±1, all four ankle joints, and a URDF `<limit>` for every joint. No network. |
| `on_configure` | Confirms network and motors, motors stay disabled. 1. **Route:** the socket's local address must be on the board's subnet (`192.168.10.x`), else the wired port is down and packets would go out via Wi-Fi → fail. 2. **Link:** 50 `ENABLE = 0` packets, logs replies and RTT → fail if none answers. 3. **Motor data:** waits up to 1 s for the board to switch to CONTROL (`NO_CM4_DATA` cleared) → fail if not (e.g. nucleotool still connected). 4. **Motor status table:** per joint online / position vs URDF limits / temperature / quickStatus faults, then "Ready to enable" or a warning listing what activation will refuse. Missing motors don't fail configure, so states stay visible for diagnosis. |
| *inactive* | `read()` runs (ros2_control calls it in this state), sends an `ENABLE = 0` keep-alive each cycle so the board keeps answering; `write()` is not called. States and RViz work, motors stay disabled. |
| `on_activate` | 1. `ENABLE = 0` for 300 ms (fresh states; the firmware only enables after it has seen `ENABLE = 0`). 2. Refuse unless the board has motor data, all 12 motors are online without quickStatus error, and every joint is inside its URDF limits. 3. Commands = current pose, velocity / torque = 0. 4. Send `ENABLE \| GAINS_VALID` with the configured kp / kd until the board reports `WBC_ST_ENABLED` (max 1 s; the board writes and verifies the gains first), else disable and fail. The motors then hold that pose with those gains. Blocks ~0.4 s. |
| *active* | `write()` sends the mapped joint PVT commands with `ENABLE` every cycle. `read()` returns `ERROR` when the board leaves `ENABLED` (command timeout, fault, damping) or after `timeout_cycles` cycles without a reply; controller_manager then deactivates the hardware. |
| `on_deactivate` | 3 × `ENABLE = 0` at once; the inactive keep-alive takes over. |
| `on_error` | 3 × `ENABLE = 0`, close the socket. |
| `on_cleanup` | Close the socket. |

The single-turn MD80 encoder is the reason for the start range: after power-on a joint position is
only correct within one rotor turn (about ±18–30° at the output), so a joint reading outside its
URDF range is probably wrong and must not be enabled.

`read()` / `write()` and the lifecycle callbacks share the socket through a mutex; the loop only
`try_lock`s and skips a cycle while a transition holds it, so it never blocks.

Board-side safety (firmware, independent of this plugin): enable only on an `ENABLE` edge with all
12 motors stable; 20 ms without a command → damping (target = measured position, the motor's own
kd) for 1 s → disable; the same on
bus-off, motor fault or NaN; the MD80's own CAN watchdog (100–200 ms).

### Diagnostics

- `NO_CM4_DATA`: warned once. The states are placeholders because the CM4 control loop is not
  running.
- `CMD_TIMEOUT`: warned at most once per second.
- Per motor: offline and `quickStatus` error groups (main / output / calibration encoder, mosfet
  bridge, hardware, communication, motion; bit names from CANdle-SDK `MDStatus.hpp`), collected
  into one message per second.
- Missed replies (`MISSED`: the motor's reply was lost on CAN, its command still arrived, the state
  is one cycle old) are normal now and then (mostly MD 60, about 1 in 30 000 cycles). They are only
  counted, at the end of the 5 s timing line (`missed replies MD 60:2`). A warning appears when a
  motor misses 3 replies in a row; the board damps and disables at 10 in a row.

---

## Build and run

```bash
cd ~/Documents/Programs/biped_wbc_mpc_ros_ws
./build.sh                                   # builds biped_description + biped_hardware
source install/setup.bash

ros2 launch biped_bringup hardware.launch.py              # with RViz, motors stay disabled
ros2 launch biped_bringup hardware.launch.py enable:=true # motors enabled right after startup, holding the pose
ros2 launch biped_bringup hardware.launch.py rviz:=false
```

Launch arguments: `enable` (default `false`), `model` (URDF in `biped_description/urdf`, default
`v2_biped_robot_12dof.urdf`), `board_ip`, `rviz`, `base_height` (RViz only).

`enable:=true` runs `ros2 control set_hardware_component_state BipedSystem active` once
`joint_state_broadcaster` is up. A refused activation (motor offline, joint outside its limits)
leaves the hardware inactive and ROS running; the log says why. Ctrl+C deactivates the hardware
(motors disabled) before shutting down.

Network: the PC's wired port must be `192.168.10.1/24`. Check that `ip route get 192.168.10.10`
uses that port and not Wi-Fi.

### Checks

```bash
ros2 control list_hardware_components      # BipedSystem: inactive after launch
ros2 control list_hardware_interfaces
ros2 topic hz /joint_states                # ~500 Hz
ros2 topic echo /joint_states --once       # match values by name: the order is not fixed
```

### Enable the motors and hold a pose (robot hanging, legs free)

```bash
ros2 control set_hardware_component_state BipedSystem active     # enabled, holds the current pose
                                                                 # (or launch with enable:=true)
ros2 control set_hardware_component_state BipedSystem inactive   # motors disabled
```

The motors hold the pose they had at activation with the gains from `config/joint_gains.yaml`,
until a controller sends new position targets. Set the gains there first: they decide how stiff
the robot is the moment it is enabled (all 0 = limp).

### Move joints with sliders (read / write check)

```bash
ros2 launch biped_bringup hardware.launch.py slider:=true
```

Activates the hardware, spawns `joint_jogging_controller` (package
[`biped_controllers`](../biped_controllers/README.md), runs inside the 500 Hz loop), then opens
`joint_state_publisher_gui` (remapped to `/slider_joint_states`, since `/joint_states` is the real
robot):

- every target starts at the measured pose; a joint follows its slider only after that slider has
  been moved (the GUI starts at 0, untouched joints keep holding their pose)
- targets move at most `max_velocity` rad/s, then pass a 0.1 s low-pass filter (both in
  `biped_bringup/config/controllers.yaml`, `joint_jogging_controller`)
- position targets only (velocity 0, effort 0); the motors track them with the configured kp / kd
- Ctrl+C deactivates the hardware (motors disabled)

### Related tools

- `STM32H755_WS_FDCan_UDP/FDCanUdp/pc/wbc_udp_test.py`: UDP rate / loss / latency test with the
  same packets (never enables unless `--enable`).
- **nucleotool** (`CANdle-SDK-udp`, UDP port 9998): discover, zero, calibration, register access,
  encoder readout. Only works while the board is in MAINTENANCE mode, and is kept separate from
  this plugin on purpose.

---

## Simulation (MuJoCo)

`biped_bringup/launch/sim.launch.py` runs the same ros2_control setup with `mujoco_ros2_control` in **PVT mode**
instead of `BipedSystem`: same component name, joints, interfaces (`BipedSystem/enabled`
included), lifecycle (starts inactive and limp, activate = hold the current pose) and safety
(20 ms command timeout → damping 1 s → disabled, `read()` error). So `controllers.yaml` and the
controllers in `biped_controllers` run unchanged. Details and the robot / sim comparison:
`mujoco_ros2_control/README.md`, section "Biped PVT Mode".

```bash
ros2 launch biped_bringup sim.launch.py                  # hanging robot, motors disabled (limp)
ros2 launch biped_bringup sim.launch.py enable:=true     # enabled, holding the pose
ros2 launch biped_bringup sim.launch.py slider:=true     # enabled + sliders (jogging)
```

- Scene: `biped_description/mjcf/scene_hanging_12dof_v2.xml`, the v2 robot with its torso welded
  1.0 m above the floor (keyframe `hang`). The floating-base scene comes next.
- The sim ankle is a **serial** roll / pitch chain: `ankle_kinematics.hpp` is only used on the
  robot, and the ankle kp / kd act on the joints directly in sim.
- Gains: section `robot` of `config/joint_gains.yaml`, as the robot; the ankle joints take the sum
  of the two ankle motors' gains (`parallel_ankles`). The MJCF links are light, so keep kd low (see
  the file).
- The controller manager runs on the simulation clock (`use_sim_time`, `/clock` from MuJoCo).
