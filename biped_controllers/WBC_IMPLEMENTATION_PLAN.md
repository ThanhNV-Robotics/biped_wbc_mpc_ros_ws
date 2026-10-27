# WBC controller – implementation plan

Status of 2026-10-27. Part A: what is done, in the order we did it. Part B: what comes next, in
the order to do it. Each milestone names the lifecycle method it belongs to and ends with a check.

Goal: `biped_controllers/WBCController`, a ros2_control controller that, every 500 Hz cycle, reads
the joint states and the IMU, estimates the floating-base state and the foot contacts, plans the
CoM / foot references (LIP), solves a TSID whole-body QP and sends PVT commands (position,
velocity, feed-forward torque) to the 12 joints.

```
load          configure          activate                        run (500 Hz)        deactivate
on_init  -->  on_configure  -->  *_interface_configuration  -->  update  ... update  -->  on_deactivate
                                 on_activate
```

Two layers:
- **`biped_wbc_core`** (no ROS): the algorithms – `StateEstimator`, `ContactFusion`,
  `LIPTaskPlanner`, `QP_TSID`. Testable with small standalone programs.
- **`biped_controllers/WBCController`** (ros2_control): the order of the calls and the controller
  logic. Helpers next to it in `wbc_controller/` (A6): `ControllerParams` (parameters, paths),
  `JointMap` (ros2_control ↔ Pinocchio order), `RobotInterface` (claim, bind, read, write, hold).
  Lifecycle steps are protected virtual methods (`setupWbc`, `setupStateEstimate`,
  `resetModules`, `estimate`, `computeCommand`), so a variant (e.g. a cheater estimator from the
  simulator's ground truth) only overrides one of them.

Run: `ros2 launch biped_bringup tsid_wbc.launch.py` (MuJoCo floating base, BipedSystem active from
startup; until the WBC writes commands the motors hold the keyframe pose). After every C++ change:

```bash
./build.sh --packages-select biped_wbc_core biped_controllers
nm -C -u install/biped_controllers/lib/libbiped_controllers.so | grep WBCController   # must be empty
ros2 node list | grep controller_manager    # before launching: must be empty (no leftover sim)
```

---

# Part A – Done (in the order we did it)

### A1. Infrastructure
- [x] Plugin entry `biped_controllers/WBCController` in `biped_controllers.xml`.
- [x] `biped_bringup` package: launch `tsid_wbc.launch.py` (joint_state_broadcaster →
      imu_sensor_broadcaster → wbc_controller, robot_state_publisher), `controllers.yaml`
      (`wbc_controller` type, joints, URDF), `tsid_wbc.yaml` (LIP planner, contacts, tasks).
- [x] Sim IMU: `<sensor name="imu">` in `biped_hw.urdf.xacro` (MJCF `baselink-quat / -gyro /
      -baseAcc`), PVT-mode log cleanup in `mujoco_ros2_control`.

### A2. Core modules (`biped_wbc_core`)
- [x] `KalmanFilter` (+ `set_Q`, `set_R`, `set_state`, `get_P`).
- [x] `StateEstimator`: 15-state KF, TSID `RobotWrapper` with its own `pinocchio::Data`,
      `update(sensor, contact_probability)`, `getPinocchioState(q, v)`. Noise parameters
      (`StateEstimatorParams`) are placeholders → tune in B3.
- [x] `ContactFusion`: momentum observer → foot wrenches, sole height, adaptive ground, contact
      probability KF + hysteresis; loop with the estimator with a one-tick delay.
- [x] `LIPTaskPlanner`: port of `WalkPlanner.py`; exact LIP integration and closed-loop ZMP (the CP
      lands on each foothold); outputs `getRef*`, `getStanceLeg`, `getPhase`, `getDt`; `reset()`,
      `setVelocityCommand(vx)`. Visual test: `ros2 run biped_wbc_core test_lip_task_planner`.
- [x] `QP_TSID` (part): free-flyer robot from the URDF, `InverseDynamicsFormulationAccForce`,
      contacts parsed from the yaml (since A7 on the controller's robot model, see below).

### A3. `on_init()` – parameters
- [x] `urdf_package`, `urdf_file`, `config_package`, `config_file` (renamed in A6), `joints`,
      `left_foot_frame`, `right_foot_frame`, `imu_name`, `enabled_interface` (`try / catch`, log).

### A4. `on_configure()` – build everything once
- [x] Re-read all parameters (a `ros2 param set` between load and configure takes effect).
- [x] `InitControlModules()` (since A6: `setupWbc()` + `setupStateEstimate()`): paths via `ament_index_cpp`; `QP_TSID` → `LIPTaskPlanner` →
      `StateEstimator`, `ContactFusion` on the QP_TSID robot; contact fusion soles = TSID contacts;
      `dt` = `1 / sampling_frequency` of the yaml (`get_update_rate()` is 0 in Humble; keep it equal
      to the controller_manager `update_rate` = 500).
- [x] `BuildJointMap()` (since A6: `JointMap::build`): 12 names, each an actuated 1-DoF joint of the model, no duplicates →
      `ros_to_pin_` / `pin_to_ros_` (tested with shuffled, wrong, duplicate and short lists).
- [x] IMU helper `imu_` (`semantic_components::IMUSensor`, since A6 inside `RobotInterface`).
- [x] `AllocateBuffers()`: `sensor_`, `q_ (19)`, `v_ (18)`, `contact_probability_ (2)`, `dv_ (18)`,
      `tau_ (12)`, `q_cmd_`, `v_cmd_`, `tau_cmd_ (12)`.

### A5. IMU through the lifecycle
- [x] `state_interface_configuration()`: the 10 `imu/...` interfaces (type `INDIVIDUAL`).
- [x] `on_activate()`: `assign_loaned_state_interfaces` (error if missing), first reading logged;
      `contact_probability_ = [1, 1]`.
- [x] `on_deactivate()`: `release_interfaces()`.
- Checked in the sim: 10 IMU interfaces claimed; orientation ≈ (0, 0.004, 0, 1),
  acceleration ≈ (−0.07, 0, 9.81).

### A6. Refactor: helpers + virtual lifecycle steps (2026-10-27)
- [x] `ControllerParams` (`controller_params.hpp/.cpp`): `declare(node)` in `on_init`,
      `read(node)` in `on_configure` (checks, resolves `urdf_path` / `config_path`). Parameters
      renamed `wbc_config_package` → `config_package`, `wbc_tsid_yaml` → `config_file`.
- [x] `JointMap` (`joint_map.hpp/.cpp`, no ROS): `build(model, joints, error)`, `toPin` / `toRos`,
      `toPinOrder` / `toRosOrder` (no allocation), `describe()`.
- [x] `RobotInterface` (`robot_interface.hpp/.cpp`): interface names, `bind` / `release`,
      `motorsEnabled`, `read(sensor)`, `holdCurrentCommand` (bumpless), `holdMeasured`,
      `write(q, v, tau)` in **Pinocchio order**, `writeLastCommand`.
- [x] `WBCController`: lifecycle + `setupWbc`, `setupStateEstimate`, `resetModules`, `estimate`,
      `computeCommand` (protected virtual); `update()` = motors check → read → estimate →
      computeCommand → write (or hold the last command).
- Checked in the sim: same logs, 36 commands claimed, deactivate / re-activate bumpless.

### A7. Robot model owned by the controller (2026-10-27)
- [x] `WBCController::setupRobotModel()` loads the URDF as a free-flyer `RobotWrapper`
      (`robot_model_`, declared before the modules); `on_configure`: parameters → robot model →
      joint map → `setupWbc` → `setupStateEstimate` → `RobotInterface` → buffers.
- [x] `QP_TSID(robot, yaml, urdf)` takes the model by reference (the formulation `tsid_` is a
      plain member); `getRobotWrapper()` removed. An estimator-only or MPC controller now gets the
      model without QP_TSID.
- [x] Sole corners: free function `utils::soleCornersFromUrdf(urdf, link, normal, corners)`
      (`biped_wbc/common/urdf_geometry.hpp`), used by QP_TSID and the contact fusion setup.
      `urdfdom` declared as a dependency of `biped_wbc_core`.
- Every module keeps its own `pinocchio::Data`; only the model is shared.
- Checked in the sim: same logs, no "No sole" warning, base z 0.860 steady, bumpless re-activate.

---

# Part B – Next (in order)

### B1. Joint interfaces – `*_interface_configuration()` + `on_activate()` (lifecycle step 3)
- [x] `command_interface_configuration()`: `INDIVIDUAL`, for each joint of `joint_names_`:
      `position`, `velocity`, `effort` (36).
- [x] `state_interface_configuration()`: add for each joint `position`, `velocity`, `effort` (36)
      and `enabled_interface_`.
- [x] `on_activate()`: find every loaned joint interface **by name** (`get_prefix_name()` /
      `get_interface_name()`, as `JointJoggingController`), store the indices; fail if one is
      missing; refuse if `enabled` is 0 (motors off).
- [x] Hold command, **bumpless**: keep the position command the drives already hold (measured q
      only if it is NaN), `v = 0`, `τ = 0`. Not `q_cmd_ = q`: under gravity the PD hold lags its
      command (ankle ≈ 0.007 rad) and that lag is the holding torque; `p = q` dropped it, the robot
      sagged and rocked (2–10° pitch).
- **Check**: `ros2 control list_controllers -v`, `ros2 control list_hardware_interfaces` → 36
  commands `[claimed]`.

### B2. Read / write – `update()` (lifecycle step 4, first version: hold the pose)
- [x] Joints → `sensor_` in Pinocchio order: `RobotInterface::read` via `JointMap`
      (also `dqj`, `torquej`).
- [x] IMU → `sensor_.imu_sensor`: `get_orientation()` is {x, y, z, w} → `Quat(q[3], q[0], q[1], q[2])`,
      `get_angular_velocity()`, `get_linear_acceleration()`.
- [x] Write `q_cmd_`, `v_cmd_`, `tau_cmd_` to the command interfaces.
- Rules for `update()`: no allocation, no file / parameter access, `RCLCPP_*_THROTTLE` only.
- [x] Motors disabled in `update()` → `holdMeasured()` (error throttled, return OK: on Humble an
      `ERROR` from update() does not stop the controller); NaN / Inf reading → last command.
- **Check**: switching the controller on → the robot keeps standing exactly as before (done:
  quaternion ≈ (1, 0, 0.005, 0), acceleration ≈ (−0.1, 0, 9.81)).

### B3. Estimation, still holding the pose – `update()` + reset in `on_activate()`
- [x] `on_activate()` → `resetModules()`: `StateEstimator::reset(sensor)` (new: base above the
      origin at the height where the soles touch z = 0, feet from leg kinematics, v = bias = 0,
      P0; `setBasePosEst` alone left the filter's feet at the origin) and `ContactFusion::reset()`
      (new: probability 1, observer / ground re-seeded; also used by the constructor). The first
      reading is now checked (NaN → activation fails).
- [x] `update()` → `estimate()`: `state_estimator_->update(sensor_, contact_probability_)` →
      `getPinocchioState(q_, v_)` → `computeDisturbanceTorque(q_, v_, measured τ)` →
      `contact_probability_ = fuseContactProbability(scheduled_contact_, contact_phase_)`
      (members, both feet / phase 0.5 until the planner runs).
- Sim (standing, straight legs): base z 0.8608 vs ground truth 0.8600, x within ±6 mm, velocity
  < 1 cm/s, clean restart on re-activation. **Foot force ≈ 0 N** and contact probability 0.53:
  with straight knees the leg Jacobian is singular (σ_min = 0, a 100 N vertical force gives 0 N·m
  at every joint), so f_z is not observable from joint torques. Recheck with bent knees (B6 / B8).
- [x] Real-time: the per-cycle estimation calls allocate nothing (was 73 allocations / cycle:
      `StateEstimator::update` 21, `getPinocchioState` 4, `computeDisturbanceTorque` 28,
      `fuseContactProbability` 20). `KalmanFilter`: preallocated work buffers, `noalias()`, LLT
      solve instead of `inverse()`, `Eigen::Ref` inputs, `get_state()` by reference.
      `StateEstimator`: `y_` member, `getPinocchioState` from the members. `ContactFusion`:
      `p_` / `St_` / `x_` members, fixed-maximum-size leg matrices + SVD, sole height loop.
      Guard: gtest `test_estimator_alloc` (counts every malloc, per call, over 1000 cycles; same
      result before / after). Reuse it for QP_TSID and the planner.
- [x] Debug topic `/wbc_controller/estimate` (`std_msgs/Float64MultiArray`, 50 Hz,
      `realtime_tools::RealtimePublisher` created in `setupDebugPublisher()`, filled in
      `publishDebug()` with `trylock`, message sized once). Layout (header + `layout.dim[0].label`):
      base position [0-2], base velocity world [3-5], contact probability [6-7], f_z [8-9],
      sole height [10-11], contact state [12-13], accelerometer bias [14-16].
- [x] **Check** vs `/simulator/floating_base_state` (standing): velocity error ≈ 0; z +0.8 mm
      (MuJoCo soft contact: the soles sink ≈ 0.8 mm, the estimator puts them at z = 0; the URDF
      sole depth 0.04 m = `foot_sole_offset`); x −4.8 mm constant (estimator origin = robot
      position at activation, simulator origin = spawn point).
- [ ] With the viewer: push the robot, base velocity follows ground truth (PlotJuggler
      `/wbc_controller/estimate/data[3]` vs `/simulator/floating_base_state/twist/twist/linear/x`).
      Σ f_z ≈ 203 N and contacts 1 1 only with bent knees (B6). Tune `StateEstimatorParams` then.

### B4. Leave safely – `on_deactivate()` (lifecycle step 5)
- [ ] Open (decide later): what to command on deactivation. Now: release only, the last command
      stays and the next controller takes it over bumplessly. A final `p = q` hold would drop the
      gravity lag (sag, see B1).
- **Check**: `switch_controllers --deactivate wbc_controller --activate position_controller` and
  back: no jump (re-add `position_controller` to `controllers.yaml` for this test).

### B5. `QP_TSID` complete (`biped_wbc_core`) + standalone test
- [x] `Contact6d` left / right (`left_foot_contact_task`, `right_foot_contact_task`): sole corners
      from the URDF collision box, normal, μ, f_min / f_max, kp / kd; `w_force_reg` stored.
- [x] Tasks created from `tsid_wbc.yaml` (weight / priority stored in `TaskSetting`): `com_task`
      (`TaskComEquality`), `base_orientation_task` (`TaskSE3Equality` on `link0_torso`, mask),
      `posture_task` (`TaskJointPosture`), `actuation_bounds_task` (`TaskActuationBounds`, limits
      from the URDF `<limit effort>`, yaml `tau_max` optional override), swing tasks
      (`TaskSE3Equality` per foot, world frame).
- [ ] Add them to the formulation: `addRigidContact(contact, w_force_reg)` for both feet,
      `addMotionTask(task, weight, priority)`, `addActuationTask(actuation_bounds, weight, 0)`.
- [ ] Solver `SOLVER_HQP_EIQUADPROG_FAST`, resized once.
- [ ] API: `setCoMReference(pos, vel, acc)`, `setFootReference(i, ...)`, `setPostureReference(q_j)`,
      `setBaseOrientationReference(R)`, `solve(t, q, v, dv_out, tau_out) -> bool`.
- [ ] Contact switching: `removeRigidContact(name, contact_transition_time)` (swing task on) /
      `addRigidContact` (swing task off).
- **Check** (standalone program): standing at home → τ ≈ gravity compensation, contact forces
  sum to m g.

### B6. Command 1 – homing, posture task only – `update()`
- [ ] Motion state for the five commands (extend `MotionState` in `data_type.hpp`).
- [ ] Both contacts, posture task to the home joint angles (quintic ramp), CoM / orientation tasks
      off or low weight; TSID references from the current state at activation (no jump).
- [ ] Command: `v_des = v_j + dv_j dt`, `q_des = q_j + v_des dt`; `p = q_des`, `v = v_des`,
      `τ_ff = τ` (Pinocchio → ros2_control order with `pin_to_ros_`).
- [ ] **Gains**: the drive adds `kp (p − q) + kd (v − dq)`; the hold gains (ankle 80) fight the
      TSID torques → low-gain file for WBC mode (`joint_gains_sim_wbc.yaml`, launch `gains:=`).
- [ ] Safety (from now on, all commands): solver failure, NaN / Inf, |τ| over `tau_max`, torso
      tilt > ~30°, both contacts lost while standing → hold the last valid `q_cmd_` with `τ = 0`.
- **Check**: the robot goes to home and stands; small pushes recovered.

### B7. Joystick – `on_configure()` + `update()`
- [ ] Subscription `/humanoid/walking_velocity_command` (`humanoid_mpc_msgs/WalkingVelocityCommand`)
      into a `realtime_tools::RealtimeBuffer` (add `humanoid_mpc_msgs` to the dependencies); read
      it in `update()`; timeout → zero velocity; button → motion command.

### B8. Command 2 – init walking: lower the CoM to `com_height` – `update()`
- [ ] CoM task on, reference from the current CoM down to `zc` (quintic ramp), base orientation
      upright, posture low weight; at the end `lip_task_planner_->reset(com, left_foot, right_foot)`.

### B9. Planner before walking (`biped_wbc_core`)
- [ ] Double support between steps (the ZMP jumps at each switch now).
- [ ] Cubic Bezier in x (zero foot velocity at liftoff / landing).
- [ ] Clamp the ZMP to the support polygon; step adjustment for the rest.

### B10. Command 3 – walking – `update()`
- [ ] Each cycle: `setVelocityCommand(vx)` → `planWalking()` → CoM and foot references to
      `QP_TSID` → contact switching from `getStanceLeg()`; contact fusion inputs `s_phi`, `phi`
      from the planner; early / late touchdown from the fused contact state.

### B11. Command 5 – walking → stop, then command 4 – turning
- [ ] Stop: finish the current step, both feet down, CP to the middle of the feet, back to
      command 2 / 1.
- [ ] Turning: yaw rate → foot orientation and footholds in the planner, yaw reference of the base
      orientation task.

### B12. Real-time check
- [ ] Cycle time (estimation / planner / TSID / total, budget < 1 ms).
- [ ] `update()` allocates nothing: replace the by-value returns used in the loop
      (`KalmanFilter::get_state()`, `StateEstimator::getEstimatedRobotState()`,
      `getBasePosEst()`) by references / output arguments.

### B13 (optional). `on_cleanup()`
- [ ] Reset the `unique_ptr`s in reverse order (IMU, fusion, estimator, planner, QP_TSID) so
      `on_configure` can run again without unloading.

---

### Real robot (after the sim works)
The IMU is not in the UDP protocol yet (no `imu/...` interfaces on hardware: activation fails
there by design). The parallel ankle is mapped by `BipedSystem`, the WBC stays in joint space.
Test order: hanging robot with estimation only → homing (posture only) → standing with a rope.
