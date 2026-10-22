# biped_hardware

ros2_control hardware interface (`hardware_interface::SystemInterface`) for the 12-motor biped.
The PC talks over Ethernet UDP to a **NUCLEO-H755ZI-Q** board, which drives the 12 MAB MD80 motor
drivers over two CAN FD buses.

> **Current stage: read path only.** The plugin reads states, but it never enables the motors:
> `write()` sends packets with `ENABLE = 0`. See [NOTES.md](NOTES.md) for the status and next steps.

---

## System overview

```
 PC (ROS 2 Humble)                                   NUCLEO-H755ZI-Q                       Motors
 ┌──────────────────────────────────┐   UDP :9999    ┌──────────────┐  SRAM4   ┌──────────┐  FDCAN1  ┌───────────┐
 │ controller_manager (500 Hz)      │  cmd 256 B ──▶ │ CM7          │ mailbox  │ CM4      │ ───────▶ │ left leg  │
 │   read() ─ update() ─ write()    │                │ LwIP, UDP    │ ───────▶ │ 1 kHz    │          │ IDs 10-60 │
 │ BipedSystem (this package)       │ ◀── state 224 B│ server       │ ◀─────── │ CAN loop │  FDCAN2  ┌───────────┐
 └──────────────────────────────────┘                └──────────────┘          └──────────┘ ───────▶ │ right leg │
   192.168.10.1                                        192.168.10.10                                 │ IDs 70-120│
                                                                                                     └───────────┘
```

- **PC:** `controller_manager` runs `read()`, the controllers' `update()` and `write()` at 500 Hz.
- **CM7:** answers each command packet with one state packet.
- **CM4:** runs the CAN cycle at a fixed 1 kHz and applies the latest command. A slow or missing PC
  never stalls the motor loop.

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
├── config/controllers.yaml    controller_manager rate and controllers
├── launch/hardware.launch.py  real robot bring-up
└── biped_hardware.xml         pluginlib description (biped_hardware/BipedSystem)
```

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

### Joint space ↔ motor space

ros2_control only sees the 12 URDF joints. `BipedSystem` converts in `read()` (and, from step 5,
in `write()`).

- **Hips and knees:** `joint = direction · motor` (position, velocity and torque).
- **Parallel ankles:** each ankle has two motors that move roll and pitch together.
  - The `*_ankle_roll_joint` carries the upper motor (50 / 110) and the `*_ankle_pitch_joint`
    carries the lower motor (60 / 120), same convention as the earlier `humanoid_hardware`.
  - `direction` is applied to each motor first, then `ankle_kinematics.hpp` maps the pair:
    - position: `left/rightJointAngles` (forward map, Newton iteration, clamped to ±0.349 rad)
    - velocity: `footVelFromMotor` (inverse of the motor Jacobian)
    - torque: `ankleTorqueFromMotor` (Jacobian transpose)
  - Both ankle motors read 0 when the ankle is level, so no offset is needed (checked:
    round trip error 3e-15 over ±0.3 rad, Jacobian matches finite differences for both legs).

Per-joint parameters in the URDF `<ros2_control>` block:

| Parameter | Meaning |
|---|---|
| `motor_index` | index into `motors[]` (0–11), each used once |
| `direction` | `1` or `-1`, so a positive motor angle matches the positive URDF joint angle |

Hardware parameters: `board_ip` (`192.168.10.10`), `board_port` (`9999`), `timeout_cycles` (`10`).

### Interfaces

| Per joint | Interfaces |
|---|---|
| State | `position`, `velocity`, `effort` |
| Command | `position`, `velocity`, `effort`, `kp`, `kd` |

The command interfaces match the drive's impedance law, so a whole-body controller can send a
desired position, velocity, feed-forward torque and gains per joint.

### Lifecycle and safety

| Callback | Does |
|---|---|
| `on_init` | Parse parameters. Require exactly 12 joints, unique `motor_index`, `direction` ±1 and all four ankle joints. No network. |
| `on_configure` | Open the socket. Send `ENABLE = 0` packets every 2 ms until the board answers (max 200 ms), otherwise fail. Fill the states from the first reply. |
| `on_activate` | Drop replies queued since configure. Set position commands to the current positions, all other commands and gains to 0. |
| `on_deactivate` | Send 3 `ENABLE = 0` packets (write() is not called any more). |
| `on_error` | Send 3 `ENABLE = 0` packets and close the socket. |
| `on_cleanup` | Close the socket. |
| `read` | Newest state → joint values. After `timeout_cycles` cycles in a row without a reply, return `ERROR`; controller_manager then deactivates the hardware. |
| `write` | Step 2: `ENABLE = 0` keep-alive. |

Board-side safety (firmware): motors disabled at boot, a command timeout with a safe action, and
the MD80's own CAN watchdog (100–200 ms).

Still to add on the PC side before enabling (step 5): refuse to activate unless all 12 motors are
online and each joint is inside its start range. The MD80 encoder is single-turn on the rotor, so
after power-on a joint position is only correct within one rotor turn (about ±18–30° at the output).

### Diagnostics

- `NO_CM4_DATA`: warned once. The states are placeholders because the CM4 control loop is not
  running.
- `CMD_TIMEOUT`: warned at most once per second.
- Per motor: offline, missed reply and `quickStatus` error groups (main / output / calibration
  encoder, mosfet bridge, hardware, communication, motion; bit names from CANdle-SDK
  `MDStatus.hpp`), collected into one message per second.

---

## Build and run

```bash
cd ~/Documents/Programs/biped_wbc_mpc_ros_ws
./build.sh                                   # builds biped_description + biped_hardware
source install/setup.bash

ros2 launch biped_hardware hardware.launch.py              # with RViz
ros2 launch biped_hardware hardware.launch.py rviz:=false
```

Launch arguments: `model` (URDF in `biped_description/urdf`, default `v2_biped_robot_12dof.urdf`),
`board_ip`, `rviz`, `base_height` (RViz only).

Network: the PC's wired port must be `192.168.10.1/24`. Check that `ip route get 192.168.10.10`
uses that port and not Wi-Fi.

### Checks

```bash
ros2 control list_hardware_components      # BipedSystem: active
ros2 control list_hardware_interfaces
ros2 topic hz /joint_states                # ~500 Hz
ros2 topic echo /joint_states --once
```

### Related tools

- `STM32H755_WS_FDCan_UDP/FDCanUdp/pc/wbc_udp_test.py`: UDP rate / loss / latency test with the
  same packets (never enables unless `--enable`).
- **nucleotool** (`CANdle-SDK-udp`, UDP port 9998): discover, zero, calibration, register access,
  encoder readout. Only works while the board is in MAINTENANCE mode, and is kept separate from
  this plugin on purpose.
