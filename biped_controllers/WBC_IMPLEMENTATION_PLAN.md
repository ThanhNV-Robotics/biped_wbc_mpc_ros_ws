# WBC controller – implementation plan

Goal: `biped_controllers/WBCController`, a ros2_control controller that, every 500 Hz cycle, reads
the joint states and the IMU, estimates the floating-base state and the foot contacts
(`biped_wbc_core`), solves a TSID whole-body QP and sends PVT commands (position, velocity,
feed-forward torque) to the 12 joints.

Work in small steps. Each step ends with something you can check in the MuJoCo floating-base sim
(`ros2 launch biped_hardware sim_floating.launch.py`) before going on. Steps 1–4 only **watch**
(the robot is held at HOME); the WBC starts commanding torques in step 5.

```
read interfaces → RobotSensor → StateEstimator ─┐            ┌→ TSID (tasks, contacts) → τ, dv
        ▲                          ▲            │ q, v       │                            │
        │                          └ p_contact ─┴→ ContactFusion → contact state ─────────┘
        └────────── write PVT: p = q_des, v = v_des, τ_ff = τ ◄── integrate dv ◄──────────┘
```

---

## Step 0 – Loadable skeleton

- [ ] Stubs for `on_init`, `command_interface_configuration`, `state_interface_configuration`
      (see the end of the last review) so the vtable is complete.
- [ ] Entry `biped_controllers/WBCController` in `biped_controllers.xml`.
- [ ] `controllers.yaml`: add under `controller_manager`
      `wbc_controller: {type: biped_controllers/WBCController}` and an empty
      `wbc_controller: ros__parameters:` section.
- **Check**
  - `nm -C -u install/biped_controllers/lib/libbiped_controllers.so | grep WBCController` → empty
  - `ros2 control list_controller_types` shows it
  - `ros2 control load_controller --set-state inactive wbc_controller` works

## Step 1 – Parameters and interfaces

- [ ] `on_init`: declare parameters (`auto_declare`), e.g.
  - `joints` (12 names), `left_foot_frame`, `right_foot_frame`
  - `imu_name` (sensor prefix, step 2), `enabled_interface` (`BipedSystem/enabled`)
  - `estimator.*`, `contact_fusion.*`, `tsid.*` gains / weights (later steps)
- [ ] `command_interface_configuration`: `INDIVIDUAL`, 12 × `position`, `velocity`, `effort`.
- [ ] `state_interface_configuration`: 12 × `position`, `velocity`, `effort`, the 10 IMU interfaces
      (step 2), `BipedSystem/enabled`.
- [ ] `on_activate`: find each loaned interface **by name** (as `JointJoggingController` does), store
      the indices; fail if one is missing.
- [ ] `update`: copy states into a preallocated `RobotSensor`; write `p = q_measured`, `v = 0`,
      `τ = 0` (holds the pose, like the activation hold).
- **Check**: `ros2 control switch_controllers --deactivate position_controller --activate
  wbc_controller` → the robot keeps standing; `ros2 control list_controllers -v` shows the claims.

## Step 2 – IMU in the simulation

The hardware exports no IMU yet. The MJCF already has the sensors on site `imu` (torso origin,
torso axes): `baselink-quat`, `baselink-gyro`, `baselink-baseAcc` (and `lf-touch` / `rf-touch`).

- [ ] In `biped_hardware/urdf/biped_hw.urdf.xacro`, inside the `use_sim` block of
      `<ros2_control>`, add a `<sensor>` (see `mujoco_ros2_control/README.md` §3.5) with
      `mujoco_type imu`, `mujoco_sensor_name baselink` and the suffix params
      `orientation_mjcf_suffix -quat`, `angular_velocity_mjcf_suffix -gyro`,
      `linear_acceleration_mjcf_suffix -baseAcc`, plus the 10 state interfaces
      (`orientation.x/y/z/w`, `angular_velocity.*`, `linear_acceleration.*`).
- [ ] Not verified yet with PVT mode: check that the sensor interfaces appear
      (`ros2 control list_hardware_interfaces`).
- **Check** (log at a few Hz from the controller, or a temporary broadcaster): orientation
  matches `/simulator/floating_base_state` (watch the quaternion order: Eigen
  `Quaterniond(w, x, y, z)`), gyro ≈ 0 and accel ≈ (0, 0, +9.81) when standing still.
- **Real robot**: the IMU is not in the UDP protocol yet (open question in
  `biped_hardware/NOTES.md`). It blocks steps 4+ on hardware, not in sim.

## Step 3 – Robot model

- [ ] `on_configure`: get the URDF (`get_robot_description()` of the controller, or a
      `urdf_path` parameter if your ros2_control version lacks it) and build
      `robot_ = std::make_unique<tsid::robots::RobotWrapper>(urdf, {}, pinocchio::JointModelFreeFlyer())`.
      With the URDF *string*, build the `pinocchio::Model` with `pinocchio::urdf::buildModelFromXML`
      and use the `RobotWrapper(model, FLOATING_BASE_SYSTEM)` constructor.
- [ ] Map ros2_control joint order → Pinocchio order by name (`model.names[2..]`; index 0 =
      universe, 1 = free flyer). Build it once; use it every cycle (estimator, fusion, TSID all
      expect Pinocchio order).
- [ ] Check: nq 19, nv 18, both foot frames exist, total mass ≈ 20.7 kg.

## Step 4 – State estimation and contact fusion (still holding HOME)

- [ ] `on_configure`: create `state_estimator_` and `contact_fusion_` from `robot_->model()`
      (dt = 1 / update rate); `setSoleCorners` for both feet (v2 foot box in the ankle pitch frame:
      x ∈ [-0.05, 0.13], y = ±0.03, z = -0.04).
- [ ] `on_activate`: reset (re-create or re-init) both; `p_contact = [1, 1]`; seed the base
      position with `setBasePosEst` (e.g. from leg kinematics, base z ≈ 0.846 at HOME).
- [ ] `update`: the loop from `DEV_LOG.md`:
      `se.update(sensor, p_contact)` → `getPinocchioState(q, v)` →
      `cf.computeDisturbanceTorque(q, v, τ_measured)` →
      `p_contact = cf.fuseContactProbability(s_phi, phi)` (standing: `{true, true}`, `{0.5, 0.5}`).
- [ ] Debug output: a `realtime_tools::RealtimePublisher` (e.g. base pose / velocity estimate,
      contact probabilities, foot forces) published at 50–100 Hz, not every cycle.
- **Check** against `/simulator/floating_base_state` (PlotJuggler):
  - base height / velocity follow the ground truth; drift over 1 min standing
  - push the robot in the MuJoCo viewer (Ctrl + right drag): velocity estimate follows
  - foot forces sum ≈ m g (≈ 203 N), contact = 1 1
  - verify that the effort state in PVT mode is the applied joint torque (input of the observer)
- Tune `StateEstimatorParams` here (current values are placeholders).

## Step 5 – TSID standing balance (first torque commands)

- [ ] `on_configure`: `InverseDynamicsFormulationAccForce`, then
  - `Contact6d` per foot: sole corners, contact normal (0, 0, 1), friction μ ≈ 0.6–0.8,
    f_min > 0, f_max ≈ 3 m g; Kp / Kd for the contact motion task
  - `TaskComEquality` (CoM over the support polygon), `TaskSE3Equality` on `link0_torso`
    (orientation; position weight 0 or small), `TaskJointPosture` (HOME, low weight)
  - torque bounds from the MJCF `ctrlrange` (hip / knee 48, hip yaw 16, ankles 18 Nm)
  - solver: `SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, ...)`
- [ ] `on_activate`: references from the **current** state (CoM, torso pose, posture = measured q),
      so nothing jumps; then move them to the targets with `utils::quinticTrajectory` (1–2 s).
- [ ] `update`: `computeProblemData(t, q, v)` → `solve` → `getAccelerations` (dv),
      `getActuatorForces` (τ). Command:
      - `v_des = v_j + dv_j * dt`, `q_des = q_j + v_des * dt` (joint part of the integration)
      - write `p = q_des`, `v = v_des`, `τ_ff = τ` (map back to ros2_control order)
- [ ] **Gains**: the drive applies `kp (p − q) + kd (v − dq)` with fixed gains while active. The
      HOME-hold gains (ankle 80) fight the TSID torques. Plan a separate gains file for WBC mode
      (e.g. `joint_gains_sim_wbc.yaml`, low kp, small kd) and a launch argument; gains change only
      when the hardware is re-activated.
- [ ] **Safety** in `update`: solver failure, NaN / Inf, |τ| over the limit, base tilt > ~30°,
      contact lost on both feet while standing → hold the last valid `q_des` with `τ = 0` and stay
      there (log once, not every cycle). The drive already damps after a 20 ms command gap.
- **Check**: switch position_controller → wbc_controller while standing: no jump; stands for
  minutes; small pushes recovered; CoM reference moved ±2 cm → follows; then lower / raise the
  pelvis with the CoM height.

## Step 6 – Real-time hygiene (before walking, before hardware)

- [ ] No allocation in `update`: preallocate every `Eigen::VectorXd`, the `RobotSensor`, message
      objects; reuse TSID / Pinocchio data. Check with a quick `malloc` counter or valgrind once.
- [ ] Measure the cycle time (`std::chrono::steady_clock` around estimator / TSID / total; publish
      max and mean at 1 Hz). Budget at 500 Hz: < 1 ms for the whole update.
- [ ] No `RCLCPP_INFO` per cycle (`RCLCPP_*_THROTTLE` only).

## Step 7 – Walking (later)

- [ ] Command input: subscribe `/humanoid/walking_velocity_command`
      (`humanoid_mpc_msgs/WalkingVelocityCommand`, from `joyStick/remote_control`) into a
      `realtime_tools::RealtimeBuffer`; read it in `update`. Scale the normalized [-1, 1] values;
      timeout → zero velocity.
- [ ] Gait scheduler: phase φ ∈ [0, 1] and scheduled contact per foot (fixed step time, double
      support ratio). It also provides `s_phi`, `phi` to `ContactFusion`.
- [ ] Footstep / CoM planner: LIPM + capture point first; MPC later (`biped_mpc`?).
- [ ] TSID: swing foot `TaskSE3Equality` with a swing trajectory; contact switch with
      `removeRigidContact(name, transition_time)` / `addRigidContact` at touchdown — trigger from
      the **fused** contact state (early / late touchdown), not only the schedule.
- **Check**: stepping in place → forward walking → the 1.5 cm step box in the scene
  (`step_box`, x = 0.63 … 1.63 m).

## Step 8 – Real robot

- [ ] IMU on the robot (protocol / hardware decision first).
- [ ] Ankle: the robot's ankles are a parallel linkage; `BipedSystem` maps joint ↔ motor, the WBC
      works in joint space (as in sim). Check torque limits in joint space against the two motors.
- [ ] Order: hanging (`fixed_base` scene first in sim, then the robot in a gantry) with the WBC
      only estimating → posture task only → standing with a safety rope.
- [ ] Gains for the robot's WBC mode (`joint_gains.yaml` variant), checked on the hanging robot.

---

### Files you will touch

| File | Steps |
|---|---|
| `biped_controllers/include/wbc_controller/wbc_controller.hpp`, `src/wbc_controller/wbc_controller.cpp` | all |
| `biped_controllers/biped_controllers.xml` | 0 |
| `biped_hardware/config/controllers.yaml` | 0, 1, 4, 5 |
| `biped_hardware/urdf/biped_hw.urdf.xacro` | 2 |
| `biped_hardware/config/joint_gains_sim_wbc.yaml` (new) + launch argument | 5 |
| `biped_wbc_core/...` (TSID task setup, gait scheduler as ROS-free classes) | 5, 7 |

Keep ROS out of `biped_wbc_core`: the controller converts interfaces ↔ `RobotSensor` / Eigen, the
core does the math. That keeps the algorithms testable with small standalone programs (as done for
the estimator).
