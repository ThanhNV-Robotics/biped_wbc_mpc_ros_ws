# WBC controller – implementation plan

Status of 2026-10-26. Part A: what is done, in the order we did it. Part B: what comes next, in
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
- **`biped_controllers/WBCController`** (ros2_control): parameters, interfaces, the order of the
  calls, conversions interfaces ↔ `RobotSensor` / Eigen.

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
      contacts parsed from the yaml; getters `getRobotWrapper()`, `getContactInfo()`.

### A3. `on_init()` – parameters
- [x] `urdf_package`, `urdf_file`, `wbc_config_package`, `wbc_tsid_yaml`, `joints`,
      `left_foot_frame`, `right_foot_frame`, `imu_name`, `enabled_interface` (`try / catch`, log).

### A4. `on_configure()` – build everything once
- [x] Re-read all parameters (a `ros2 param set` between load and configure takes effect).
- [x] `InitControlModules()`: paths via `ament_index_cpp`; `QP_TSID` → `LIPTaskPlanner` →
      `StateEstimator`, `ContactFusion` on the QP_TSID robot; contact fusion soles = TSID contacts;
      `dt` = `1 / sampling_frequency` of the yaml (`get_update_rate()` is 0 in Humble; keep it equal
      to the controller_manager `update_rate` = 500).
- [x] `BuildJointMap()`: 12 names, each an actuated 1-DoF joint of the model, no duplicates →
      `ros_to_pin_` / `pin_to_ros_` (tested with shuffled, wrong, duplicate and short lists).
- [x] IMU helper `imu_` (`semantic_components::IMUSensor`).
- [x] `AllocateBuffers()`: `sensor_`, `q_ (19)`, `v_ (18)`, `contact_probability_ (2)`, `dv_ (18)`,
      `tau_ (12)`, `q_cmd_`, `v_cmd_`, `tau_cmd_ (12)`.

### A5. IMU through the lifecycle
- [x] `state_interface_configuration()`: the 10 `imu/...` interfaces (type `INDIVIDUAL`).
- [x] `on_activate()`: `assign_loaned_state_interfaces` (error if missing), first reading logged;
      `contact_probability_ = [1, 1]`.
- [x] `on_deactivate()`: `release_interfaces()`.
- Checked in the sim: 10 IMU interfaces claimed; orientation ≈ (0, 0.004, 0, 1),
  acceleration ≈ (−0.07, 0, 9.81).

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
- [x] Hold command from the current state: `q_cmd_ = q`, `v_cmd_ = 0`, `tau_cmd_ = 0`.
- **Check**: `ros2 control list_controllers -v`, `ros2 control list_hardware_interfaces` → 36
  commands `[claimed]`.

### B2. Read / write – `update()` (lifecycle step 4, first version: hold the pose)
- [ ] Joints → `sensor_` in Pinocchio order: `sensor_.actuator_state.qj(ros_to_pin_[i]) = ...`
      (also `dqj`, `torquej`).
- [ ] IMU → `sensor_.imu_sensor`: `get_orientation()` is {x, y, z, w} → `Quat(q[3], q[0], q[1], q[2])`,
      `get_angular_velocity()`, `get_linear_acceleration()`.
- [ ] Write `q_cmd_`, `v_cmd_`, `tau_cmd_` to the command interfaces.
- Rules for `update()`: no allocation, no file / parameter access, `RCLCPP_*_THROTTLE` only.
- **Check**: switching the controller on → the robot keeps standing exactly as before.

### B3. Estimation, still holding the pose – `update()` + reset in `on_activate()`
- [ ] `on_activate()`: estimator base position from leg kinematics (`setBasePosEst`).
- [ ] `update()`: `state_estimator_->update(sensor_, contact_probability_)` →
      `getPinocchioState(q_, v_)` → `contact_fusion_->computeDisturbanceTorque(q_, v_, τ)` →
      `contact_probability_ = contact_fusion_->fuseContactProbability({true, true}, {0.5, 0.5})`.
- [ ] Debug publisher (`realtime_tools::RealtimePublisher`, 50–100 Hz, created in `on_configure`):
      base estimate, contact probabilities, foot forces.
- **Check** vs `/simulator/floating_base_state` (PlotJuggler): base height / velocity, Σ foot
  forces ≈ 203 N, contacts 1 1, pushes in the viewer. Tune `StateEstimatorParams`.

### B4. Leave safely – `on_deactivate()` (lifecycle step 5)
- [ ] Final hold `p = q`, `v = 0`, `τ = 0` before releasing (the IMU release is done).
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
