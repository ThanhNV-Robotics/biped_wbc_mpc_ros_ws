# legged_wbc — Whole-Body Controller

Converts the MPC output (centroidal state + contact forces) into joint torques via full rigid-body dynamics, solved as a QP at 500 Hz.

---

## Role in the Stack

```
MPC (~80 Hz)
  optimizedState(t)   → desired centroidal state over horizon
  optimizedInput(t)   → desired contact forces [F_left(3), F_right(3)]
          │
          ▼
    legged_wbc (500 Hz)
          │
          ├── updateMeasured()   — run pinocchio FK/dynamics on measured state
          ├── updateDesired()    — run pinocchio FK/dynamics on MPC desired state
          ├── formulate tasks    — build QP blocks
          └── solve QP (qpOASES)
                    │
                    ▼
          joint torques τ (12)  → command_interfaces effort
```

---

## Decision Variables

Every control cycle the WBC solves for a single vector `x` of size `generalizedCoordinatesNum + 3×numContacts + actuatedDofNum`:

```
x = [ q̈ (18),   F_contact (6),   τ (12) ]
      ──────     ────────────     ──────
      base+joint  foot forces     joint torques
      accel       (3 per foot)    ← final output
```

For our biped: `18 + 6 + 12 = 36` decision variables per cycle.

---

## Tasks

Each task is a struct `Task{A, b, D, f}` encoding:
- **Equality**: `A·x = b`
- **Inequality**: `D·x ≤ f`

Tasks can be stacked with `+` and scaled with `*`.

| Task | Type | What it enforces |
|------|------|-----------------|
| `formulateFloatingBaseEomTask` | equality | `M·q̈ - J^T·F - S^T·τ = -h`  (Newton-Euler EOM — the physics must be satisfied) |
| `formulateTorqueLimitsTask` | inequality | `‖τ‖ ≤ τ_max` per joint |
| `formulateNoContactMotionTask` | equality | contact foot acceleration = 0 (foot in contact must not move) |
| `formulateFrictionConeTask` | inequality | contact forces inside friction pyramid (`μ = frictionCoeff`) |
| `formulateBaseAccelTask` | objective | track the base acceleration implied by the MPC centroidal input |
| `formulateSwingLegTask` | objective | PD-track swing foot position/velocity from MPC desired state |
| `formulateContactForceTask` | objective | track the MPC-desired contact forces as a reference |

### Floating Base EOM detail

```
[M  -J^T  -S^T] · [q̈; F; τ] = -h

M   — joint-space inertia matrix          (from pinocchio::crba)
J   — contact foot Jacobian               (from pinocchio::getFrameJacobian)
S   — selection matrix [0 | I_actuated]   (maps τ to actuated DOF only)
h   — nonlinear effects (Coriolis+gravity) (from pinocchio::nonLinearEffects)
```

This is the core constraint: any solution must satisfy Newton-Euler dynamics.

### Swing Leg Task detail

```
accel_des = Kp × (pos_des - pos_meas) + Kd × (vel_des - vel_meas)
J_swing · q̈ = accel_des - J̇_swing · q̇
```

`pos_des` / `vel_des` come from the MPC desired state (via pinocchio FK on the optimal trajectory). This is how the leg swing motion is generated — the MPC sets where the foot should be; the WBC computes the joint torques to get it there.

---

## Solver Variants

### `WeightedWbc` (default)

Combines all tasks into a single weighted QP:

```
Hard constraints (equality + inequality):
  FloatingBaseEOM + TorqueLimits + FrictionCone + NoContactMotion

Soft objectives (weighted least-squares):
  weightSwingLeg    × SwingLegTask
  weightBaseAccel   × BaseAccelTask
  weightContactForce × ContactForceTask

min  ‖W·(A·x - b)‖²
s.t. constraints
```

Solved with qpOASES (`QProblem`, MPC mode, max 20 iterations).

### `HierarchicalWbc`

Uses `HoQp` (Hierarchical Optimization QP) to enforce tasks in strict priority order — higher-priority tasks are never sacrificed for lower-priority ones. More principled than weights but slower.

---

## Output

The QP solution vector `x = [q̈, F, τ]`. The WBC caller extracts:

```cpp
vector_t torques = qpSol.tail(info.actuatedDofNum);  // last 12 elements = joint torques
```

These are written to the `feedforward_effort` command interfaces, combined with the PD position hold:

```
final torque = Kp × (pos_cmd - pos) + Kd × (vel_cmd - vel) + τ_wbc
```

---

## Configuration (`task.info`)

```
torqueLimitsTask
{
  (0,0)  33.5   ; hip
  (1,0)  33.5   ; knee
  (2,0)  33.5   ; ankle
}

frictionConeTask
{
  frictionCoefficient  0.3
}

swingLegTask
{
  kp  350.0
  kd    37.0
}

weight
{
  swingLeg      100.0
  baseAccel       1.0
  contactForce    0.01
}
```

---

## Build Status

The C++ source is complete and correct. The `CMakeLists.txt` and `package.xml` are still in **ROS1 (catkin)** format and must be ported to **ROS2 (ament_cmake)** before the package can be built.
