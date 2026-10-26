# Development log

Newest entries first. Per-package details: `biped_hardware/NOTES.md`; how the launch files work: `biped_bringup/SIMLAUNCH.md`.

---

## 2026-10-27

### QP_TSID tasks (`biped_wbc_core`)
- `ContactInfo` removed; left / right handled separately. `setupContactTask` (Contact6d per foot,
  sole corners from the URDF collision box, μ / f_min / f_max / kp / kd), `setupSwingTask`
  (TaskSE3Equality per foot, world frame), `setupCoMTask`, `setupBaseOrientationTask`,
  `setupPostureTask` (kp 10 added to the yaml), `setupActuationBoundsTask`. Tasks are created, not
  yet added to the formulation; weight / priority stored in `TaskSetting`.
- Torque limits from the URDF `<limit effort>` (`model.effortLimit`); `v2_biped_robot_12dof.urdf`
  updated from the placeholder 1000 to 48 / 48 / 16 / 48 / 18 / 18 N·m. yaml `tau_max` = optional
  override.

### WBCController: B2 + bumpless activation
- `update()` reads joints (Pinocchio order) + IMU every cycle and writes the hold command; motors
  disabled → hold the measured pose; NaN → last command.
- Found: activating with `q_cmd = measured q` dropped the PD gravity lag (ankle ≈ 0.007 rad ≈
  0.5 N·m) → the robot sagged and rocked 2–10° in pitch. Fix: keep the position command the drives
  already hold. Now < 1° and steady.

### WBCController refactor (after reviewing `legged_control`, ROS 1)
- Helpers in `biped_controllers/wbc_controller/`: `ControllerParams`, `JointMap`, `RobotInterface`;
  lifecycle steps as protected virtual methods (`setupWbc`, `setupStateEstimate`, `resetModules`,
  `estimate`, `computeCommand`). `wbc_controller.cpp` 568 → ~330 lines, same behaviour.
- Parameters renamed: `wbc_config_package` → `config_package`, `wbc_tsid_yaml` → `config_file`.
- `src/legged_control` (ROS 1 clone for reference) has a `COLCON_IGNORE`.

---

## 2026-10-25

### New package `biped_bringup` (launch files + system configuration)

- Moved from `biped_hardware` to `biped_bringup`: `launch/{hardware,sim,sim_floating,tsid_wbc}.launch.py`,
  `config/controllers.yaml`, `config/controllers_sim_floating.yaml`, `scripts/odom_to_tf.py`,
  `SIMLAUNCH.md`. Start everything with `ros2 launch biped_bringup <file>.launch.py`.
- Stays in `biped_hardware`: the `BipedSystem` plugin, `urdf/biped_hw.urdf.xacro` and the joint
  gains (`config/joint_gains*.yaml`, read by the xacro; keeps `$(find biped_hardware)` working and
  the hardware package independent of the bringup package).
- Launch files use `hw_share` (xacro, gains) and `bringup_share` (controllers yaml, scripts).
- Run dependencies of the launch files (controllers, MuJoCo, broadcasters, RViz, ...) moved from
  `biped_hardware/package.xml` to `biped_bringup/package.xml`. Docs and comments updated
  (READMEs, SIMLAUNCH, WBC plan, MJCF comments); the old `NOTES.md` entries are kept as written.
- `build/` + `install/` of `biped_hardware` rebuilt once (stale launch files in `install/`).
- Tested: all 4 launch files load; `tsid_wbc` (joint_state_broadcaster, imu_sensor_broadcaster,
  wbc_controller active), `sim enable:=true` and `sim_floating` start, hardware active.
- **Open:** `sim_floating.launch.py` still spawns `position_controller`, which is no longer in
  `controllers.yaml` → "type param was not defined". Re-add it to the yaml or drop the spawner +
  HOME publish from the launch file.

### Earlier today
- Sim IMU: `<sensor name="imu">` in `biped_hw.urdf.xacro` (MJCF `baselink-quat/-gyro/-baseAcc`,
  suffix params in the `<hardware>` block); `imu_sensor_broadcaster` in `controllers.yaml`.
- `tsid_wbc.launch.py`: joint_state_broadcaster → imu_sensor_broadcaster → wbc_controller, plus
  robot_state_publisher; RViz and the `wbc_active` option removed (kept simple).
- `mujoco_ros2_control`: `is_pvt_mode()`; in PVT mode the pd_gains_file block and the per-joint
  actuator-mode messages are skipped (no more "no PD gains loaded yet" warnings).
- `WBCController::on_init()` (step 1 of `biped_controllers/WBC_IMPLEMENTATION_PLAN.md`, rewritten
  by lifecycle step): parameters `urdf_package`, `urdf_file`, `joints`, foot frames, `imu_name`,
  `enabled_interface`; joint list in `controllers.yaml`. Next: step 2 `on_configure()` (add
  `ament_index_cpp` to `biped_controllers` first).

---

## 2026-10-23

### 1. State estimator and contact fusion (`biped_wbc`, renamed `biped_wbc_core` in section 3)

Ported the state estimator and the contact fusion model from the earlier project into
`biped_wbc`, on top of TSID / Pinocchio, and closed the loop between them.

**Package setup**
- `CMakeLists.txt` / `package.xml`: one shared library `biped_wbc` (exported as
  `biped_wbc::biped_wbc`), C++17, Release by default.
- Dependencies: Eigen3, pinocchio 4.0, tsid 1.10 (apt `ros-humble-*`), yaml-cpp. MuJoCo removed
  (no longer used by `common/utils.hpp`).
- Sources: `common/utils.cpp`, `state_estimation/{kalman_filter, StateEstimator, contact_fusion}.cpp`.
- Headers are included as `<biped_wbc/...>`. Fixed stale includes from the old project
  (`common/data_type.h`, `common/utils.h`, `estimation/contact_fusion.h`,
  `state_estimation/kalman_filter.hpp`).

**`KalmanFilter`** (generic linear KF, used by both estimators)
- Added `set_Q`, `set_R` (time-varying noise), `set_state` (overwrite the estimate without
  resetting P), `get_P`; `get_state()` const.
- `update()` forces P symmetric; state zeroed at construction; fixed the `-Wreorder` warning.

**`StateEstimator`** (`state_estimation/StateEstimator.{hpp,cpp}`)
- Linear KF, 15 states `[p_B, v_B, p_foot_L, p_foot_R, accel_bias]` (world frame), 14
  measurements `[p_B - p_foot (2x3), v_B (2x3), p_foot.z (2)]`. Orientation and angular velocity
  from the IMU, joint states passed through. Filter model unchanged from the old project.
- Uses the package `KalmanFilter` (was hand-written inside the class).
- Uses TSID's `tsid::robots::RobotWrapper` (was the old project's own `RobotWrapper`):
  - constructor `StateEstimator(robot, dt, left_foot_frame, right_foot_frame, params, verbose)`,
    throws for a fixed-base model or an unknown frame
  - foot kinematics with Pinocchio on `robot.model()`, base at the origin, in its **own**
    `pinocchio::Data` (never touches the data of the TSID formulation)
  - the old `rb_wrapper.updateRobotState(*this)` is gone (TSID's wrapper holds no state); the
    estimate is read out with `getEstimatedRobotState()` (`RobotState`, world-frame twist) or
    `getPinocchioState(q, v)` (nq 19 / nv 18, TSID convention, via `utils::robotStateToPinocchio`)
- Noise / offset parameters in `StateEstimatorParams`.
- Contact input: `update(sensor, contact_probability)` takes the per-foot contact probability
  from `ContactFusion` and scales Q / R continuously (1x stance ... 100x swing). The touch-sensor
  sigmoid path was removed.

**`ContactFusion`** (`state_estimation/contact_fusion.{hpp,cpp}`, Bledt et al. ICRA 2018)
- Generalized-momentum disturbance observer → per-foot wrench (damped least squares on
  `S_i J_i^T`), sole height from the sole corners, adaptive ground height.
- Contact probability KF (package `KalmanFilter`, A = 0, B = I, C = [I; I]) fusing the phase
  prior with the height and force probabilities, then hysteresis (p_on 0.5 / p_off 0.4).
- Header pasted from the old project; only the include paths changed. Tuned values kept
  (paper covariances, force variance lowered to 0.5 from sim replay).

**Estimator ↔ fusion loop** (each needs the other's output → one-tick delay)
```cpp
Eigen::VectorXd p_contact = Eigen::VectorXd::Ones(2);   // 1. start in double support
// every control tick:
se.update(sensor, p_contact);                            // 2. estimator, previous tick's p
se.getPinocchioState(q, v);
cf.computeDisturbanceTorque(q, v, sensor.actuator_state.torquej);
p_contact = cf.fuseContactProbability(s_phi, phi);       // 3. fusion → 4. used next tick
```
The delay is 2 ms at 500 Hz, small against 10–30 ms touchdown / liftoff. Standing without a
gait scheduler: `s_phi = {true, true}`, `phi = {0.5, 0.5}`.

**Tests** (standalone program against the installed library, v2 URDF, free-flyer, home pose
`[-0.2, 0, 0, 0.4, 0, -0.2]` per leg, IMU at rest, static joint torques from `g(q) - J^T f`)

| Case | Contact probability L / R | Contact | fz L / R [N] | Base estimate |
|---|---|---|---|---|
| Double support (mg/2 = 101.6 N per foot) | 0.99 / 0.99 | 1 1 | 91 / 91 | z 0.846 m (MuJoCo home 0.848), v ≈ 0 |
| Left foot unloaded, scheduled swing (not lifted) | 0.29 / 0.99 | 0 1 | 0 / 182 | unchanged |

Also checked: an unknown foot frame and a wrong-size contact vector throw.

**Open points**
- `StateEstimatorParams` defaults are **placeholders** (Cheetah-style noise, sole offset 0.04 m
  from the MJCF foot box): the old header with the tuned values was lost. Tune in sim.
- Observer force about 10 % low at knee 0.4 rad, probably the wrench damping (0.02) on a nearly
  straight leg. No effect on detection (threshold 20 N); matters if the force value is used.
- Unloading-only test: in a real swing the foot also rises, so the height channel will lower the
  probability further. Test in the MuJoCo floating-base sim with real swing motion.
- Joint order: `RobotSensor::actuator_state` must be in Pinocchio order (left hip pitch, roll,
  yaw, knee, ankle roll, ankle pitch, then right) = `controllers.yaml` order, **not**
  `/joint_states` order; map by name in the controller.
- `KalmanFilter::update` uses `.inverse()` of the innovation covariance; an LDLT solve would be
  cheaper and more robust (14 x 14 every tick).
- `common/utils.hpp`: `using namespace std;` in a header; leaks into every includer.

### 2. Build setup and VS Code IntelliSense

- New `build.sh` in the colcon workspace root (`legged_robot_ros2_ws/`): `colcon build
  --symlink-install -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, then merges every
  `build/<pkg>/compile_commands.json` into `build/compile_commands.json`. Extra arguments go to
  colcon (`./build.sh --packages-select biped_wbc_core`).
- `.vscode/c_cpp_properties.json` (in `legged_robot_ros2_ws/` and in `biped_wbc_mpc_ros_ws/`, for
  either folder opened): IntelliSense from that file, fallback include path for files not built
  yet. `.vscode/tasks.json`: Ctrl+Shift+B runs `build.sh`, plus a single-package task.
- Whole workspace switched to `--symlink-install`: `build/` + `install/` of `biped_controllers`,
  `mujoco_ros2_control`, `mujoco_ros2_control_msgs` deleted once (copy → symlink conflict), then
  all 9 packages rebuilt without warnings.
- After adding a .cpp / changing a CMakeLists: run `./build.sh` so IntelliSense gets the flags.

### 3. Package layout: `biped_wbc_core` + `biped_controllers`

- `biped_wbc` renamed `biped_wbc_core` (folder, `package.xml`, `project()`); old build / install
  output removed. Export: `find_package(biped_wbc_core)`, `biped_wbc_core::biped_wbc_core`.
  Headers still under `include/biped_wbc/` (`#include "biped_wbc/..."`).
- Split decided: `biped_wbc_core` = ROS-free algorithms (Eigen, Pinocchio, TSID);
  `biped_controllers` = ros2_control plugins, depends on `biped_wbc_core`.
- `wbc_controller.hpp/.cpp` moved from `biped_wbc_core` to
  `biped_controllers/{include,src}/wbc_controller/`; its includes now
  `"biped_wbc/state_estimation/..."`.
- `biped_controllers`: fixed the paths of the moved jogging controller
  (`src/joint_jogging_controller/joint_jogging_controller.cpp`, include
  `"joint_jogging_controller/joint_jogging_controller.hpp"`).

### 4. WBC controller skeleton

- `WBCController` (`biped_controllers`) compiles: `tsid::robots::RobotWrapper`, `StateEstimator`,
  `ContactFusion` held as `std::unique_ptr` (created in `on_configure` from the URDF; pluginlib
  needs a default constructor). `robot_` declared first (the estimators reference its model).
- **Not loadable yet**: `nm -C -u libbiped_controllers.so` shows an undefined
  `vtable for biped_controllers::WBCController` — `on_init`,
  `command_interface_configuration`, `state_interface_configuration` are still missing
  (stubs given), and there is no entry in `biped_controllers.xml` yet.
- Step-by-step plan: `biped_controllers/WBC_IMPLEMENTATION_PLAN.md` (steps 0–8: skeleton →
  interfaces → sim IMU → robot model → estimation while holding HOME → TSID standing →
  real-time hygiene → walking → robot). The user codes it, starting with step 0.

**Next session**
- Step 0 of the plan: stubs + plugin XML + `controllers.yaml` entry; check with `nm` and
  `ros2 control load_controller`.
- Step 2 needs the IMU `<sensor>` in `biped_hw.urdf.xacro` (MJCF names `baselink-quat`,
  `-gyro`, `-baseAcc` → suffix params); not yet tested together with PVT mode.
- Stale `install/state_estimation` (old package, gone from `src/`) can be deleted.
