# biped_hardware — progress notes

Design: [README.md](README.md). Firmware progress: `STM32H755_WS_FDCan_UDP/FDCanUdp/NOTES.md`.
Last updated 2026-10-22.

## Status

| Step | What | State |
|---|---|---|
| 1 | `wbc_protocol.h` compiles as C++, shared copy in this package | **Done** |
| 2 | PC read path: `UdpClient`, `BipedSystem` (read only, motors never enabled), plugin, URDF block, launch | **Done**, tested against the board |
| 3 | Firmware: CM7 ↔ CM4 control-loop mailboxes, CM4 sends real motor states (not enabled) | **Done**, tested on hardware 2026-10-22; direction signs verified 2026-10-23 |
| 4 | Firmware: CM4 CONTROL mode (ENABLE, PVT / FULL + retry, timeout safe action, bus-off recovery) | **Done** 2026-10-23, tested on hardware; CM4 loop now **500 Hz** |
| 5 | PC write path: joint → motor commands (ankle inverse map + Jacobian), activation checks | **Done** 2026-10-23; tested on the robot with zero gains; gains through ROS not tested yet |

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

### 2026-10-23 — direction signs verified
- User moved every joint by hand against RViz and corrected `urdf/biped_hw.urdf.xacro`:
  `direction = -1` for **left_hip_yaw, right_hip_pitch, right_hip_yaw, right_knee_pitch**; all
  others (including both motors of both ankles) `+1`.
- Step 3 is complete. Next: step 4 decisions (timeout safe action, per-joint start range).

### 2026-10-23 — step 4 firmware written
- Decisions: timeout → damp (kd 1.0) for 1 s, then disable; enable checks in firmware **and** PC.
- Firmware side and test plan: `FDCanUdp/NOTES.md` (2026-10-23 entries). Test tool without ROS:
  `FDCanUdp/pc/wbc_enable_test.py` (limp / damp / hold, motor space).
- This package is unchanged and still never sets ENABLE; the PC side of enabling is step 5.

### 2026-10-23 — step 4 tested and done (firmware at 500 Hz)
- Enable / disable, damping on PC crash, damp and hold on one joint all work (tool:
  `FDCanUdp/pc/wbc_enable_test.py`). CM4 loop moved to 500 Hz: 1 failed cycle in 30 000.
- The firmware enables only on an ENABLE edge (`ENABLE = 0` first) with all 12 motors stable,
  and needs fresh commands (20 ms timeout → damping). Step 5 must follow this: send ENABLE = 0
  in `on_configure` / before activation, then ENABLE with targets = current pose, gains 0,
  wait for `WBC_ST_ENABLED`, then ramp the gains.

### 2026-10-23 — loop timing monitor + first measurements
- `BipedSystem` logs every 5 s: loop rate, period avg / p99 / max and cycles > 2.5 / 5 / 10 / 20 ms,
  compute time (end of `read()` → start of `write()`: the controllers, later the WBC), read / write
  max, cycles without a reply, `CMD_TIMEOUT` replies. At start: the loop thread's scheduling policy.
- PC: i7-6700 (4C/8T), kernel 6.8.0-generic `PREEMPT_DYNAMIC` (not RT), governor `powersave`,
  `ulimit -r` 0. Loop thread: **SCHED_OTHER**. Humble `ros2_control_node` only requests SCHED_FIFO
  when it detects a PREEMPT_RT kernel (`realtime_tools::has_realtime_kernel()`); it has
  `thread_priority` and `cpu_affinity` parameters.
- Results (motors disabled, joint_state_broadcaster only, 40 s each, 5 s windows):

  | Run | period p99 | period max | > 2.5 ms / 5 s | > 5 ms / 5 s | compute avg / max | no reply / 5 s |
  |---|---|---|---|---|---|---|
  | A idle desktop | 2.1–2.2 ms | 2.7–4.4 ms | 2–8 | 0 | 0.08 / 0.5 ms | 0–2 |
  | B + RViz | 2.1–2.2 ms | 3.4–5.2 ms | 4–11 | 0–1 | 0.07 / 2.7 ms | 0–4 |
  | C 8 busy threads | 4.8–5.3 ms | 6.8–10.8 ms | 168–216 | 8–28 | 0.2 / 8.9 ms | 113–171 |

  The board never hit its 20 ms timeout (only the configure → activate gap at start). Compute work
  is ~0.1–0.2 ms; the multi-ms compute values are preemptions inside `update()`.
- WBC budget: TSID biped step 0.18 ms avg / 0.8 ms worst (tsid_examples, measured at 1 kHz in sim)
  → compute fits the 2 ms cycle; scheduling jitter under load does not.
- Next: governor `performance`, rtprio/memlock limits, SCHED_FIFO for the loop (on this kernel only
  via a `chrt` launch prefix, or the `preempt=full` boot option + PREEMPT_RT kernel), then repeat
  A / B / C.

### 2026-10-23 — step 5: write path, activation, inactive start
User decisions: ankle kp / kd passed to the motors unchanged (PVT through the ankle kinematics,
coupling of the gains not handled yet); start range = URDF limits; hardware starts inactive.
- **Inactive at launch** (`hardware_components_initial_state.inactive: [BipedSystem]`). Verified:
  ros2_control calls `read()` while inactive but not `write()`, so `read()` now sends the
  `ENABLE = 0` keep-alive when not active. States / RViz work with motors disabled.
- **`on_activate`:** 300 ms `ENABLE = 0` → checks (motor data, 12 online, no quickStatus error,
  every joint inside its URDF `<limit>`, parsed from `info_.original_xml` with `urdf::Model`) →
  commands = current pose, gains 0 → `ENABLE` until `WBC_ST_ENABLED` (max 1 s). ~0.4 s blocking.
- **`write()`** (active only): hips / knees `motor = direction · joint`, ankles inverse kinematics
  (position, velocity at the commanded pose; torque via the Jacobian at the measured pose; roll /
  pitch clamped to ±0.349 rad), `ENABLE | GAINS_VALID`. Non-finite commands logged once (the
  firmware damps and disables).
- **`read()`** returns `ERROR` when the board leaves `ENABLED` while active.
- Socket shared through a mutex; `read()` / `write()` only `try_lock`.
- `config/controllers.yaml`: `position_controller`, `kp_controller`, `kd_controller`
  (ForwardCommandController, not started) for the first tests through ROS.
- `/joint_states` order: the broadcaster's `joints` parameter does **not** fix the order in Humble
  (right leg still mixed) — match by name.
- **Robot tests (zero gains only, run by Claude):** launch → inactive, states at 500 Hz;
  `set_hardware_component_state active` → ACTIVE, board `[WBC] ENABLED`, 12/12 online;
  `inactive` → board `DISABLED: ENABLE cleared by the PC`, keep-alive keeps it in CONTROL;
  active + `kill -9 ros2_control_node` → board `DAMPING ... command timeout` (`full 12`) →
  `DISABLED: damping finished` → MAINTENANCE. Loop while active: compute avg ~0.08 ms.

### Next
1. **User test with gains through ROS** (README "Enable the motors and hold a pose"): kd, then kp
   on the left knee, then the other joints one at a time; ankles last (their position targets go
   through the inverse kinematics, gains are motor-space).
2. Real-time setup (governor, rtprio, `chrt` / PREEMPT_RT) and repeat the timing runs.
3. Ankle gains in joint space; soak test; IMU.

### 2026-10-23 — `enable:=true` launch argument
- On request: `hardware.launch.py enable:=true` activates BipedSystem right after startup
  (`OnProcessExit` of the joint_state_broadcaster spawner → `ros2 control
  set_hardware_component_state BipedSystem active`). Default `false`. Chosen over
  `hardware_components_initial_state` active so a refused activation leaves the hardware inactive
  instead of failing ros2_control_node at startup.
- Tested on the robot (zero gains): ACTIVE ~0.3 s after the spawner, board `[WBC] ENABLED`;
  Ctrl+C → `deactivate` → board `DISABLED: ENABLE cleared by the PC` → MAINTENANCE after 1 s.

### 2026-10-23 — network and motor checks in `on_configure`
- `on_configure` now: route check (socket local address on 192.168.10.x, else fail — wired port
  down), 50-packet link test with RTT, wait ≤ 1 s for motor data (fail if the board stays in
  MAINTENANCE), then a motor status table (online, position vs URDF limits, temperature, faults)
  and "Ready to enable" or the list of what activation will refuse. Missing motors only warn.
- Robot: `Network OK ... 50/50 replies, RTT avg 0.17 max 0.44 ms`, 12/12 online, all inside limits.
- **MD 20 and MD 80 report 0 °C** (the two drives on fw 2.5.4; fw 2.4.x drives report ~25 °C):
  fw 2.5 likely encodes temperature differently in the 0xA0 reply. Not used for control; check
  `mab_can_protocol.md` / firmware parse before relying on temperatures.

### 2026-10-23 — slider test mode (`slider:=true`)
- `scripts/slider_to_position.py`: joint_state_publisher_gui (remapped to `/slider_joint_states`)
  → rate limit (`max_velocity`, default 0.3 rad/s) + 0.1 s low-pass → `/position_controller/commands`,
  plus constant kp / kd (defaults 5 / 0.3) to `/kp_controller`, `/kd_controller`. Starts at the
  measured pose; a joint follows its slider only after that slider moved.
- Launch chain: joint_state_broadcaster → activate → spawner (position / kp / kd controllers) →
  GUI + bridge. Launch args `slider`, `kp`, `kd`, `max_velocity`.
- Tested on the robot with `kp:=0 kd:=0` only (no torque): chain comes up, controllers active, one
  publisher each on `/joint_states` and `/slider_joint_states`, sliders picked up, clean deactivate
  on Ctrl+C (fixed a shutdown traceback in the node afterwards).
- Next: user runs it with gains, one joint at a time (ankles last).

### 2026-10-23 — PVT commands only
- User: use PVT only, the motors already have their own kp / kd. `write()` sends `ENABLE` without
  `GAINS_VALID`; kp / kd interfaces stay exported but are not used. Firmware ignores gains too
  (`WBC_WRITE_GAINS 0`) and no longer writes kp = kd = 0 at enable.
- **Activation now means: motors hold the current pose with their own gains** (stiff, not limp).
- Slider mode: positions only (`kp_controller` / `kd_controller` and the `kp:=` / `kd:=` launch
  arguments removed).
- Checked: build, flash, launch → configure report OK. Enabling with the motors' own gains not
  tested on the robot yet.
- **Ankle range check added to activation:** the ankle kinematics clamps roll / pitch to
  ±0.349 rad but the URDF allows pitch ±0.524. Beyond the clamp the motor targets built from the
  joint values would differ from the real motor positions, and with the motors' own kp the ankle
  would jump on enable. Activation now recomputes the ankle motor targets and refuses if they are
  off by more than 0.01 rad. Current pose passes (`Ready to enable`).
- Open: the ankle range ±0.349 rad of `ankle_kinematics.hpp` vs URDF pitch limit ±0.524 rad —
  decide which one is right (and widen the kinematics clamp if the mechanism allows it).

### 2026-10-23 — ankle range: roll ±15°, pitch ±30°
- Design range from the user; the URDFs (v2, original, single legs) already had exactly these
  limits — no URDF change needed.
- `ankle_kinematics.hpp`: the single clamp ±0.349 rad for roll and pitch replaced by
  `ROLL_LIMIT` 15° / `PITCH_LIMIT` 30°; Newton up to 10 iterations (converges in ~4).
- **Finding:** with the geometry constants, the inverse map has no solution in the corners of the
  box (acos argument up to 1.29): full roll ±15° only for pitch -22.7 … +26.9°, at pitch -30° only
  about ±2° roll, at +30° about ±8° (both legs mirrored). New `reachable()` and
  `clampToWorkspace()` (clamp to the box, then keep pitch and bisect the roll down) are used in
  `write()`, so ankle commands can never produce NaN. Tested: 64 000 poses incl. far outside → no
  NaN; 10 378 reachable poses unchanged by the clamp; motor ↔ joint round trip 4.6e-15.
- **Open (user):** does the real ankle reach more than the model (e.g. pitch -30° with roll ±15°)?
  If yes, the `LEFT` / `RIGHT` geometry constants are off and must be measured / checked.

### 2026-10-23 — joint_jogging_controller replaces slider_to_position.py
- New package `biped_controllers` (README there): `JointJoggingController`, a ros2_control
  controller running inside the 500 Hz loop. Same behaviour as the script (start at the measured
  pose, follow a slider only after it moved, rate limit + low-pass), writes position, velocity = 0,
  effort = 0. Slider values come through a `RealtimeBuffer` (no locks / allocation in `update()`).
- `slider:=true`: activate → spawn `joint_jogging_controller` → GUI. Script, its install rule and
  rclpy deps removed; `max_velocity` launch arg removed (now a controller parameter in
  `controllers.yaml`). `build.sh` builds `biped_controllers` too.
- **Finding:** Humble lets a controller be active while the hardware is inactive. Then moving a leg
  by hand and enabling later would make the stiff motors jump back to the controller's stale
  target. Fix: `BipedSystem` exports the state interface `BipedSystem/enabled` (1 while enabled);
  the controller tracks the measured pose while it is 0. Usable by the WBC later.
- Tested with motors disabled: plugin loads, configures, activates (finds `BipedSystem/enabled`),
  claims position / velocity / effort of all 12 joints, subscribes to `/slider_joint_states`.
  Jogging with enabled motors not tested yet (stiff hold with the motors' own gains: user first).

### 2026-10-23 — no warning for single missed replies
- User saw `Motor problems: MD 60: missed-reply` now and then (expected: about 1 in 30 000 cycles,
  lost CAN arbitration, command delivered). Removed from the warning: missed replies are counted per
  motor and appended to the 5 s timing line (`missed replies MD 60:2`); a warning only when a motor
  misses 3 replies in a row (board limit: 10).

### 2026-10-23 — gains from `config/joint_gains.yaml`, written at activation, then fixed
- **Finding:** all 12 motors had impedance kp = kd = 0 in RAM (nucleotool `md info`), left over from
  the step 4 tests (old firmware wrote 0 at every enable). "Use the motors' own gains" would have
  meant limp. Saved flash values come back only after a motor power cycle.
- New `config/joint_gains.yaml` (kp / kd per joint; ankles = motor-space gains of the motor the
  joint carries). Start values all 0 (limp) until the user fills them in. The xacro loads it
  (`xacro.load_yaml`, launch arg `gains:=`), `on_init` validates (kp 0–1000, kd 0–50),
  `on_configure` shows them in the status table and warns about kp = 0.
- `on_activate` sends `ENABLE | GAINS_VALID` with the gains; `write()` sends `ENABLE` only (PVT,
  gains fixed). kp / kd command interfaces removed (export and xacro).
- Firmware: `MD_EnableImpedance` split into `MD_PrepareImpedance` (motor disabled: targets, mode,
  optional kp / kd, read-back compare) and `MD_Enable`; `tryEnable` prepares all 12 first, then
  enables them back to back (~6 ms), logs `[WBC] gains L/R (kp/kd): ...`. Gains copied out of the
  packed packet before taking their address (unaligned float pointer on the M4 FPU).
- Robot test (test file kp 0 / kd 0.05, motors effectively limp): gains written + verified,
  ENABLED, clean disable; nucleotool afterwards: kd 0.0500 on MD 10 / 60 / 70 / 120.

### 2026-10-22 — MuJoCo simulation with the robot's interface (hanging scene)
- `mujoco_ros2_control` got a **PVT mode** (`pvt_mode` hardware param, `BipedPvtDrive`): emulates
  BipedSystem + firmware. PVT commands with fixed joint kp / kd, `BipedSystem/enabled`, inactive =
  zero torque, activation = range check + hold pose, 20 ms timeout → damping kd 1 for 1 s →
  disabled + `read()` ERROR. PD law evaluated before every `mj_step` (1 kHz).
- Found while doing it: the vendored interface's mode switch made position / velocity / effort
  exclusive (last claim wins), so `joint_jogging_controller` would have left every joint in effort
  mode at 0 Nm (limp). PVT mode has no mode switching. Also fixed two `get_clock()` calls that do
  not exist in Humble (the package did not compile on Humble).
- User decision: the sim ankle is a serial chain, no ankle kinematics in sim (joint-space gains).
- `urdf/biped_hw.urdf.xacro use_sim:=true`, `launch/sim.launch.py`, `config/joint_gains_sim.yaml`,
  `biped_description/mjcf/scene_hanging_12dof_v2.xml` (torso welded at 1.0 m, keyframe `hang`).
- Tested standalone (drive + MuJoCo 3.13, no ROS): limp hang; activation holds within 0.006 rad;
  20 ms without commands → damping → disabled. kp 100 / kd 2 stable; ankle roll chatters from
  kd ≈ 3 (tiny link inertia, explicit 1 ms step).
- Not tested through ROS yet: needs `ros-humble-mujoco-vendor` and `ros-humble-ros2-control-cmake`.
- Next: floating-base scene (standing keyframe, odometry / TF instead of the static transform).


### 2026-10-23 — floating-base simulation held at home (`launch/sim_floating.launch.py`)
- Scene `scene_floatingbase_12dof_v2.xml` got keyframe `home`: hip pitch -0.2, knee 0.4, ankle
  pitch -0.2 (both legs), torso z 0.848 so the foot boxes touch the floor.
- `BipedSystem` starts ACTIVE there (`config/controllers_sim_floating.yaml`, loaded after
  controllers.yaml): the physics runs from init, so limp motors would let the robot collapse before
  a manual activation. A launch `parameters` dict or `-p` did not override the yaml's
  `controller_manager:` section; a second yaml does.
- `position_controller` spawned after the joint_state_broadcaster, then HOME published once.
  TF world -> link0_torso from `/simulator/floating_base_state` (`scripts/odom_to_tf.py`).
- Sim ankle gains 20 / 0.3 -> 80 / 0.8: with 20 the robot (20.7 kg) tips over within 1 s on joint
  PD alone (needs 2 kp > m g h ~ 120 Nm/rad). Hanging scene not re-checked with the new gains.
- Tested headless: stands for > 35 s sim time, torso z 0.842, pitch -0.03 rad, knees sag to 0.45
  (kp 100 under load); no command timeout between the startup activation and the first write.
