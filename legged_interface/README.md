# legged_interface

The core MPC problem formulation library for the biped robot. It assembles and owns
the complete `OptimalControlProblem` used by the OCS2 MPC solver: centroidal dynamics,
quadratic tracking cost, per-foot contact constraints, self-collision avoidance, and
the gait schedule + swing trajectory planner.

This package is a pure C++ library — it has no ROS node, no topics, and no services.
It is consumed by upper-level packages (e.g., `legged_controllers`) that instantiate
`LeggedInterface`, extract the assembled `OptimalControlProblem`, and pass it to an
MPC solver (DDP, SQP, or IPM).

## State and Input Dimensions

The centroidal model has:

| Symbol | Size | Contents |
|---|---|---|
| State `x` | 24 | Normalized centroidal momentum (6) + base pose (6) + joint positions (12) |
| Input `u` | 24 | Contact forces, 3 per foot (6 for 2 feet) + joint velocities (12) |

The normalized centroidal momentum is `[h_lin / m, h_ang / m]` where `m` is total robot mass.
Base pose is `[p_x, p_y, p_z, θ_z, θ_y, θ_x]` (ZYX Euler angles).

## Required Input Files

Three files must be provided at construction time.

### 1. `task.info` — Solver and problem settings

| Section | Key parameters | Used by |
|---|---|---|
| `centroidalModelType` | `0` = Full Centroidal, `1` = SRBD | `setupModel` |
| `model_settings` | `jointNames`, `contactNames3DoF`, CppAD cache folder | `setupModel`, `getEeKinematicsPtr` |
| `model_settings.positionErrorGain` | Position feedback in normal-velocity constraint | `getZeroVelocityConstraint` |
| `swing_trajectory_config` | `swingHeight`, `liftOffVelocity`, `touchDownVelocity`, `swingTimeScale` | `SwingTrajectoryPlanner` |
| `mpc` | `timeHorizon`, `mpcDesiredFrequency`, `mrtDesiredFrequency` | `LeggedInterface` constructor |
| `ddp` | DDP solver settings | `LeggedInterface` constructor |
| `sqp` | SQP (multiple-shooting) settings: `dt`, `sqpIteration`, `nThreads` | `LeggedInterface` constructor |
| `ipm` | IPM solver settings | `LeggedInterface` constructor |
| `rollout` | `integratorType`, `AbsTolODE`, `timeStep` | `setupOptimalControlProblem` |
| `initialState` | 24-element dense matrix — cold-start guess for optimizer | `setupOptimalControlProblem` |
| `Q` | 24×24 state cost weight (diagonal) | `getBaseTrackingCost` |
| `R` | 24×24 input cost weight, transformed to task space | `getBaseTrackingCost` |
| `frictionConeSoftConstraint` | `frictionCoefficient`, `mu`, `delta` (barrier penalty) | `setupOptimalControlProblem` |
| `selfCollision` | `collisionObjectPairs`, `collisionLinkPairs`, `mu`, `delta`, `minimumDistance` | `getSelfCollisionConstraint` |

### 2. `reference.info` — Gait definition and nominal state

| Section | Key parameters | Used by |
|---|---|---|
| `defaultJointState` | Nominal joint angles (12×1) — defines the centroidal model's default config | `setupModel` |
| `defaultBaseHeight` | Standing height (m) | `SwitchedModelReferenceManager` |
| `initialModeSchedule` | `modeSequence` + `eventTimes` at startup | `loadGaitSchedule` |
| `defaultModeSequenceTemplate` | Repeating gait pattern + `switchingTimes` (defines gait period) | `loadGaitSchedule` |
| `targetDisplacementVelocity` | How fast the reference point moves toward the command | Reference manager |
| `maxDisplacementVelocityX/Y` | Velocity limits for base motion commands | Reference manager |

Mode names recognized from `ocs2_legged_robot`: `STANCE`, `WALK`, `TROT`, `LHSTAND`, `RHSTAND`, etc.
All modes encode a contact bitmask — each bit corresponds to one foot being in contact.

### 3. `robot.urdf` — Robot kinematic/dynamic model

Used to build the Pinocchio model. Only joints listed in `model_settings.jointNames` are included
as actuated DoFs; all others are locked at their URDF default positions.

## Internal Architecture

### `LeggedInterface` — top-level assembler

```
LeggedInterface(taskFile, urdfFile, referenceFile)
│   Validates file paths
│   Loads solver settings: modelSettings_, mpcSettings_, ddpSettings_, sqpSettings_,
│                           ipmSettings_, rolloutSettings_
│
└── setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose)
    │
    ├── setupModel()
    │   ├── PinocchioInterface  ←── URDF + active joint list
    │   └── CentroidalModelInfo ←── centroidal type + defaultJointState (reference.info)
    │
    ├── setupReferenceManager()
    │   ├── GaitSchedule        ←── initialModeSchedule + defaultModeSequenceTemplate (reference.info)
    │   ├── SwingTrajectoryPlanner ←── swing_trajectory_config (task.info)
    │   └── SwitchedModelReferenceManager (wraps both)
    │
    ├── OptimalControlProblem assembly:
    │   ├── Dynamics:   LeggedRobotDynamicsAD
    │   ├── Cost:       LeggedRobotStateInputQuadraticCost
    │   └── Constraints (per foot × numFeet)
    │
    ├── setupPreComputation()
    │   └── LeggedRobotPreComputation
    │
    ├── Rollout: TimeTriggeredRollout
    └── Initializer: LeggedRobotInitializer
```

### Dynamics

`LeggedRobotDynamicsAD` implements the centroidal dynamics:

```
ẋ = f(x, u) = [net_force/m; net_torque/I; v_base; joint_vel]
```

Gradients are computed via CppAD automatic differentiation and compiled to shared
libraries on first run (controlled by `recompileLibrariesCppAd` in task.info).

### Cost

`LeggedRobotStateInputQuadraticCost` minimizes:

```
J = ∫ (Δx'·Q·Δx + Δu'·R·Δu) dt
```

- `Δx = x - x_ref(t)` — deviation from the reference base trajectory (set by the MPC command interface)
- `Δu = u - u_gravity(t)` — deviation from gravity-compensating contact forces at the current contact configuration

The input weight `R` for joint velocities is transformed from task space to joint space
using foot Jacobians at the initial configuration (`initializeInputCostWeight`).

### Constraints per Foot

Four constraints are registered per foot (index `i`). Which are active depends on the
current contact mode from `SwitchedModelReferenceManager::getContactFlags(t)`:

| Constraint | Phase | Type | Formula |
|---|---|---|---|
| `ZeroForceConstraint` | Swing | Equality (linear) | `F_i = 0` — no GRF while foot is in air |
| `FrictionConeConstraint` | Stance | Inequality (quadratic) | `μ·(Fz+Fg) ≥ √(Fx²+Fy²+ε)` — GRF stays inside cone |
| `ZeroVelocityConstraintCppAd` | Stance | Equality (linear) | `v_ee = 0` — foot doesn't slide (holonomic) |
| `NormalVelocityConstraintCppAd` | Swing | Equality (linear) | `v_ee_z = ż_spline(t)` — foot tracks swing arc |

The friction cone can be used as a hard inequality constraint or wrapped in a
`RelaxedBarrierPenalty` soft constraint (controlled by `useHardFrictionConeConstraint`
flag in the `LeggedInterface` constructor).

### Self-Collision (State Constraint)

`LeggedSelfCollisionConstraint` wraps `ocs2_self_collision::SelfCollisionConstraint`
with a `RelaxedBarrierPenalty`. At each step it queries Pinocchio's collision geometry
to compute signed distances between configured link pairs and penalizes violations below
`minimumDistance`. The Pinocchio geometry interface is built from `collisionLinkPairs`
and `collisionObjectPairs` listed in the `selfCollision` section of task.info.

### Swing Trajectory Planner

`SwingTrajectoryPlanner` generates a smooth vertical foot trajectory (z-height spline)
for each swing phase using `SplineCpg` (cubic polynomial with given peak):

```
z(t): liftOff → midHeight → touchDown
midHeight = terrain_z + scaling × swingHeight

scaling = min(1.0, (t_touchDown - t_liftOff) / swingTimeScale)
```

Short swing phases are scaled down in height and velocity. The planner is updated
once per MPC solve by `SwitchedModelReferenceManager::modifyReferences()`, which
extends the mode schedule over the full optimization horizon plus margins.

### Pre-Computation (`LeggedRobotPreComputation`)

Called by the OCS2 solver at each time point before evaluating cost and constraints.
Runs Pinocchio forward kinematics and Jacobians so all constraint functions share
a single consistent kinematic state:

```
request = Approximation:
  pinocchio::forwardKinematics + updateFramePlacements + computeJointJacobians
  updateCentroidalDynamics + updateCentroidalDynamicsDerivatives

request = Constraint (without Approximation):
  pinocchio::forwardKinematics + updateFramePlacements only
  + populate eeNormalVelConConfigs_ from SwingTrajectoryPlanner
```

### Initializer (`LeggedRobotInitializer`)

Provides the OCS2 solver with a feasible initial guess:
- `input` = gravity-compensating contact forces for the current contact configuration
- `nextState` = current state (zero-order hold)
- Normalized momentum is optionally propagated or zeroed

## Information Flow

```
task.info ──────┬── Solver settings (MPC/DDP/SQP/IPM/rollout)
                ├── model_settings → joint names, contact frames, CppAD cache
                ├── swing_trajectory_config → SwingTrajectoryPlanner
                ├── initialState → cold-start x₀
                ├── Q, R → cost weights
                ├── frictionConeSoftConstraint → FrictionConeConstraint
                └── selfCollision → LeggedSelfCollisionConstraint

reference.info ─┬── defaultJointState → CentroidalModelInfo (default config)
                ├── initialModeSchedule → GaitSchedule (startup mode)
                └── defaultModeSequenceTemplate → repeating gait cycle

robot.urdf ─────── PinocchioInterface + PinocchioGeometryInterface

            ↓ LeggedInterface::setupOptimalControlProblem()

OptimalControlProblem
├── dynamicsPtr:              LeggedRobotDynamicsAD
├── costPtr:                  LeggedRobotStateInputQuadraticCost (Q·Δx², R·Δu²)
├── equalityConstraintPtr:    ZeroForceConstraint (swing)
│                             ZeroVelocityConstraintCppAd (stance)
│                             NormalVelocityConstraintCppAd (swing)
├── inequalityConstraintPtr:  FrictionConeConstraint (stance, hard mode)
├── softConstraintPtr:        FrictionCone soft (stance, barrier mode)
├── stateSoftConstraintPtr:   LeggedSelfCollisionConstraint
└── preComputationPtr:        LeggedRobotPreComputation

SwitchedModelReferenceManager
│   modifyReferences() called once per MPC solve:
│   ├── extends mode schedule over [t₀−T, t_f+T]
│   └── updates SwingTrajectoryPlanner with new schedule

LeggedRobotInitializer
    compute() called per OCP time step for warm-start input/state
```

## Package Structure

```
legged_interface/
├── include/legged_interface/
│   ├── LeggedInterface.h               # Top-level assembler class
│   ├── LeggedRobotPreComputation.h     # Per-step Pinocchio update cache
│   ├── SwitchedModelReferenceManager.h # Gait + swing trajectory manager
│   ├── constraint/
│   │   ├── EndEffectorLinearConstraint.h       # g = Ax·xee + Av·vee + b
│   │   ├── FrictionConeConstraint.h            # μ·Fz ≥ |F_tangential|
│   │   ├── LeggedSelfCollisionConstraint.h     # Minimum link distance
│   │   ├── NormalVelocityConstraintCppAd.h     # Foot z-vel tracks swing spline
│   │   ├── SwingTrajectoryPlanner.h            # SplineCpg foot height arcs
│   │   ├── ZeroForceConstraint.h               # F = 0 during swing
│   │   └── ZeroVelocityConstraintCppAd.h       # vee = 0 during stance
│   ├── cost/
│   │   └── LeggedRobotQuadraticTrackingCost.h  # Quadratic state-input cost
│   └── initialization/
│       └── LeggedRobotInitializer.h            # Gravity-compensating warm-start
├── src/
│   ├── LeggedInterface.cpp
│   ├── LeggedRobotPreComputation.cpp
│   ├── SwitchedModelReferenceManager.cpp
│   ├── constraint/
│   │   ├── EndEffectorLinearConstraint.cpp
│   │   ├── FrictionConeConstraint.cpp
│   │   ├── NormalVelocityConstraintCppAd.cpp
│   │   ├── SwingTrajectoryPlanner.cpp
│   │   ├── ZeroForceConstraint.cpp
│   │   └── ZeroVelocityConstraintCppAd.cpp
│   └── initialization/
│       └── LeggedRobotInitializer.cpp
├── CMakeLists.txt
└── package.xml
```

## Dependencies

| Package | Role |
|---|---|
| `ocs2_legged_robot` | `ModelSettings`, `GaitSchedule`, `LeggedRobotDynamicsAD`, `SplineCpg`, mode definitions |
| `ocs2_centroidal_model` | `CentroidalModelInfo`, `CentroidalModelPinocchioMapping`, helper functions |
| `ocs2_pinocchio_interface` | `PinocchioInterface`, `PinocchioEndEffectorKinematicsCppAd` |
| `ocs2_self_collision` | `SelfCollisionConstraint`, `PinocchioGeometryInterface` |
| `ocs2_core` | `OptimalControlProblem`, cost/constraint base classes, `ReferenceManager` |
| `pinocchio` | Forward kinematics, Jacobians, collision geometry |
| `Eigen3` | Matrix algebra |
| `Boost` | Filesystem (path validation), property\_tree (`.info` file parsing) |
