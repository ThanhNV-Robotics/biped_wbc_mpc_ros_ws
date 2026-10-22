# biped_hardware — progress notes

Design: [README.md](README.md). Firmware progress: `STM32H755_WS_FDCan_UDP/FDCanUdp/NOTES.md`.
Last updated 2026-10-22.

## Status

| Step | What | State |
|---|---|---|
| 1 | `wbc_protocol.h` compiles as C++, shared copy in this package | **Done** |
| 2 | PC read path: `UdpClient`, `BipedSystem` (read only, motors never enabled), plugin, URDF block, launch | **Done**, tested against the board |
| 3 | Firmware: CM7 ↔ CM4 control-loop mailboxes, CM4 sends real motor states (not enabled) | **Done**, tested on hardware 2026-10-22; direction signs still to fix |
| 4 | Firmware: CM4 CONTROL mode (ENABLE, PVT / FULL + retry, timeout safe action, bus-off recovery) | Not started |
| 5 | PC write path: joint → motor commands (ankle inverse map + Jacobian), activation checks | Not started |

## Next steps

1. **Step 3 test:** flash the firmware, then follow "Where to continue (step 3 test plan)" in
   `FDCanUdp/NOTES.md`: first motors off (mode switch, offline handling), then motors powered
   but never enabled, moving each joint by hand while watching RViz.
2. **Fix the `direction` signs** in `urdf/biped_hw.urdf.xacro` during that test. All are `1` now.
   Check the ankles separately: pure pitch and pure roll by hand should each move only one joint
   in RViz.
3. **Decide the firmware timeout action.** Proposal: damping (`kp = 0`, small `kd`, `τ = 0`) for
   0.5–1 s, then disable.
4. **Step 5:**
   - `write()`: joint commands → motor commands. Ankles: `left/rightMotorAngles` for position,
     `motorVelFromFoot` for velocity, `motorTorqueFromAnkle` for torque. Gains (kp/kd) need a
     decision for the ankle motors: joint-space gains do not map 1:1 to the two motors.
   - `on_activate`: refuse unless all 12 motors are `ONLINE` and inside a per-joint start range
     (new URDF parameters, e.g. `start_min` / `start_max`).
   - Test order: `kp = kd = 0`, then small `kd`, then small `kp` holding the current pose.

## Open questions

- **IMU:** not in the protocol. Through the Nucleo (state packet version 2) or directly to the PC?
- **Ankle gains:** how to turn joint-space kp/kd for roll / pitch into gains for the two ankle
  motors (e.g. `J^T K J` per cycle, or run the ankle impedance on the PC and send torque only).
- **Real-time:** no PREEMPT_RT kernel yet; controller_manager warns about it.

## Dev log

### 2026-10-22 — package started, read path working

**Workspace**
- OCS2 submodules and `ocs2_lib` removed from the workspace; packages moved under `src/`.
- `biped_description`: mesh paths changed to `package://biped_description/urdf/meshes/...`,
  RViz launch `display.py` (model, gui, rviz_config, base_height arguments).
- VS Code IntelliSense: `.vscode/c_cpp_properties.json` uses `build/compile_commands.json`;
  `./build.sh` builds `biped_description` + `biped_hardware` and merges the compile commands.
  STM32 clangd extension disabled for this workspace.
- `COLCON_IGNORE` in `CANdle-SDK-udp/` and `STM32H755_WS_FDCan_UDP/` (colcon picked them up as
  plain CMake packages `candlesdk` and `mdtool`).

**Step 1: protocol header**
- `wbc_protocol.h` did not compile as C++ (`_Static_assert`). Firmware header now uses
  `WBC_STATIC_ASSERT` (`static_assert` under C++). Layout unchanged. Copied byte-identical into
  `include/biped_hardware/`. Compiles with g++ C++17 `-Wpedantic -Werror`, gcc gnu11 and the
  CubeIDE arm-none-eabi-gcc 14.3.

**Ankle kinematics check** (`ankle_kinematics.hpp`, scratch test program)
- Both motors read 0 at roll = pitch = 0 for both legs → no offset parameter needed.
- Motor → joint → motor round trip: max error 2.7e-15 over ±0.3 rad.
- Analytic Jacobian matches central finite differences for the left and right leg.

**Step 2: read path**
- `UdpClient` based on the socket pattern of nucleotool's `UDP.cpp` (connected socket, poll,
  errno handling), changed to non-blocking `recv_latest()` with size / magic / version checks.
- `BipedSystem` as described in the README. Motor ↔ joint mapping follows the old
  `humanoid_hardware` (roll joint = upper ankle motor, pitch joint = lower ankle motor).
- From CANdle-SDK-udp only the socket pattern and the `quickStatus` bit names were reused; the
  library itself is not linked (it is the MAINTENANCE-mode tool, port 9998, blocking per-motor
  transfers).
- Build: no warnings with `-Wall -Wextra -Wpedantic`.
- **Board test** (firmware `a1760f1` + protocol header fix, motors not enabled):
  - init → configure → activate succeeded; board answered at once with `NO_CM4_DATA`.
  - `joint_state_broadcaster` active, `/joint_states` has all 12 joints (values 0 = placeholders).
  - Clean deactivate / shutdown on Ctrl+C; no missed-reply or send errors in the log.
  - `/joint_states` rate not measured yet (the run was interrupted). Expected ~500 Hz.
- Harmless warnings: KDL root-link inertia, deprecated `robot_description` parameter for
  `ros2_control_node`, no real-time kernel.

### 2026-10-22 (later) — step 3 firmware written
- Firmware side: see `FDCanUdp/NOTES.md` (wbc_shm.h mailboxes, CM4 `wbc_loop.c`, automatic
  MAINTENANCE ↔ CONTROL switch, motors never enabled).
- `report_status()` now logs each change of `NO_CM4_DATA` (INFO when CM4 states start, WARN
  when they stop), because the first replies after start are placeholders while CM4 pings the
  motors. Rebuilt without warnings.

### 2026-10-22 (end) — step 3 tested on hardware
- Motors off: mode switch, 1 kHz loop, ~497 Hz UDP, no lost packets. A mailbox read bug (~1 %
  placeholder replies) was found and fixed in the firmware (`WBC_SHM_READ_SPINS`); re-test clean.
- Motors powered, never enabled: all motors online, **RViz follows the robot when joints are
  moved by hand**.
- Open: some `direction` signs are wrong; user reports which joints on 2026-10-23.
