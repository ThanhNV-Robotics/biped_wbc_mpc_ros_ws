# legged_estimation

State estimation package for the biped WBC-MPC system. Estimates the full rigid-body state of the robot (base position, orientation, velocity, and joint states) from IMU measurements and foot contact kinematics using a linear Kalman filter.

---

## Package structure

```
legged_estimation/
├── include/legged_estimation/
│   ├── StateEstimateBase.h       Base class: sensor buffering, rbdState_, publishers
│   ├── LinearKalmanFilter.h      KalmanFilterEstimate: IMU + foot FK fusion
│   └── FromTopicEstimate.h       FromTopicStateEstimate: ground-truth odometry (sim cheat mode)
└── src/
    ├── StateEstimateBase.cpp     updateJointStates / updateImu / updateAngular / updateLinear
    ├── LinearKalmanFilter.cpp    update() — full Kalman predict + correct step
    └── FromTopicEstimate.cpp     update() — pass-through from /ground_truth/state
```

---

## State vectors

### `rbdState_` — full rigid-body state (output)

Size: `2 × generalizedCoordinatesNum = 2 × 18 = 36`

| Index | Contents |
|---|---|
| `[0:3]` | Base orientation — ZYX Euler angles (roll, pitch, yaw) |
| `[3:6]` | Base position — `[x, y, z]` in world frame |
| `[6:18]` | Joint positions — 12 actuated joints |
| `[18:21]` | Base angular velocity — world frame |
| `[21:24]` | Base linear velocity — world frame |
| `[24:36]` | Joint velocities — 12 actuated joints |

This is the vector returned by `update()` and consumed by `LeggedController::update()` to form the MPC observation.

### `xHat_` — Kalman filter internal state

Size: `numState_ = 6 + dimContacts_ = 6 + 6 = 12`

| Index | Contents |
|---|---|
| `[0:3]` | Base position estimate — `[x, y, z]` world frame |
| `[3:6]` | Base linear velocity estimate — world frame |
| `[6:12]` | Foot positions — 2 feet × `[x, y, z]` world frame |

---

## Kalman filter formulation

The filter fuses **IMU accelerations** (as a control input) with **foot kinematics** (as measurements) to estimate base position and velocity. Foot positions are co-estimated as part of the state.

### System model

```
xHat[k+1] = A * xHat[k] + B * accel_world
```

```
A = | I  dtI  0 |    B = | 0.5*dt²*I |
    | 0   I   0 |        | dt*I      |
    | 0   0   I |        | 0         |
```

- `accel_world = R_base_to_world * linearAccelLocal + [0,0,-9.81]` — gravity-compensated IMU acceleration
- Foot positions evolve as a random walk (process noise). Noise is scaled by `100×` when foot is not in contact.

### Observation model

```
y = C * xHat + noise
```

Observation vector size: `numObserve_ = 2 × dimContacts_ + numContacts_ = 12 + 2 = 14`

| Slice | Measurement | Source |
|---|---|---|
| `y[0:6]` | Foot positions in world frame | `base_pos - eePos_FK` |
| `y[6:12]` | Foot velocities in world frame | `-eeVel_FK` |
| `y[12:14]` | Foot height (z-coord) | `feetHeights_[i]` when in contact |

The observation matrix `C` (14 × 12) extracts position, velocity, and foot z from the state and links base position/velocity to contact point measurements.

### Noise scaling by contact state

For each foot `i`:
- If **in contact**: use nominal noise weights (low `Q_foot`, low `R_foot`)
- If **not in contact** (swing): multiply noise by `high_suspect_number = 100` — effectively ignoring foot measurements during swing

---

## Call flow (per control cycle at 500 Hz)

```
LeggedController::update(time, dt)
  │
  ├── read hardware interfaces ──────────────────────────────────────────────────
  │     joint positions, velocities, efforts
  │     IMU: orientation quaternion, angular_vel_local, linear_accel_local
  │
  ├── stateEstimate_->updateJointStates(jointPos, jointVel)
  │     rbdState_[6:18]  = jointPos
  │     rbdState_[24:36] = jointVel
  │
  ├── stateEstimate_->updateContact(contactFlag)
  │     contactFlag_[i] = true/false for each foot
  │
  ├── stateEstimate_->updateImu(quat, angVelLocal, linAccelLocal, ...)
  │     quat_ = imu quaternion
  │     ZYX = quatToZyx(quat)
  │     angVelGlobal = R * angVelLocal  (local → world frame)
  │     updateAngular() → rbdState_[0:3] = ZYX, rbdState_[18:21] = angVelGlobal
  │
  └── measuredRbdState_ = stateEstimate_->update(time, dt)
        │
        ├── Build Pinocchio config from rbdState_
        │     qPino = [0, ZYX, jointPos]   (position left at origin — filter handles it)
        │     vPino = [0, ZYX_dot, jointVel]
        │
        ├── pinocchio::forwardKinematics(qPino, vPino)
        │     → eePos[i] = foot position in world frame (relative to base at origin)
        │     → eeVel[i] = foot velocity
        │
        ├── Kalman PREDICT
        │     xHat_ = A * xHat_ + B * accel_world
        │     P     = A * P * A^T + Q
        │
        ├── Kalman CORRECT
        │     innovation  ey = y - C * xHat_
        │     S           = C * P * C^T + R
        │     K           = P * C^T * S^{-1}
        │     xHat_       += K * ey
        │     P           = (I - K * C) * P
        │
        ├── Position bias calibration (first 10s)
        │     accumulate xHat_[0:2] → compute mean x,y offset → subtract forever after
        │
        ├── updateLinear(xHat_[0:3], xHat_[3:6])
        │     rbdState_[3:6]   = base position (from filter)
        │     rbdState_[21:24] = base linear velocity (from filter)
        │
        ├── publish /odom and /pose at 200 Hz
        │
        └── return rbdState_  [36-dim full RBD state]
```

---

## Sensor inputs

| Source | Data | Updated in |
|---|---|---|
| Joint encoders | 12 × position, 12 × velocity | `updateJointStates()` |
| IMU | quaternion, angular velocity (local), linear acceleration (local) | `updateImu()` |
| Contact estimator | `contactFlag_[2]` — which feet are in contact | `updateContact()` |
| Foot kinematics | foot positions/velocities computed from FK inside `update()` | internal |

Contact flags are not directly measured — the `LeggedController` must estimate them (e.g. from foot force thresholds or from the MPC gait schedule). These flags gate the Kalman measurement noise.

---

## Parameters (`kalmanFilter` section in `task.info`)

| Parameter | Default | Description |
|---|---|---|
| `footRadius` | `0.02 m` | Foot sphere radius — added to foot z for ground contact height |
| `imuProcessNoisePosition` | `0.02` | Scales `Q[0:3, 0:3]` — position process noise |
| `imuProcessNoiseVelocity` | `0.02` | Scales `Q[3:6, 3:6]` — velocity process noise |
| `footProcessNoisePosition` | `0.002` | Scales `Q[6:12, 6:12]` — foot position process noise |
| `footSensorNoisePosition` | `0.005` | Scales `R[0:6, 0:6]` — foot position measurement noise |
| `footSensorNoiseVelocity` | `0.1` | Scales `R[6:12, 6:12]` — foot velocity measurement noise |
| `footHeightSensorNoise` | `0.01` | Scales `R[12:14, 12:14]` — foot height measurement noise |

---

## Implementations

### `KalmanFilterEstimate` (real hardware + sim)

IMU + foot contact kinematics fusion as described above. Used by `LeggedController`.

Optional external correction: subscribes to `/tracking_camera/odom/sample` (`nav_msgs/Odometry`). When a message arrives, it corrects `xHat_[0:3]` (base position) and foot positions via FK. Currently disabled (commented out) — enable by uncommenting the `topicUpdated_` block in `update()`.

### `FromTopicStateEstimate` (simulation cheat mode)

Subscribes to `/ground_truth/state` (`nav_msgs/Odometry`) and passes the odometry directly into `rbdState_`. No filtering. Used by `LeggedCheaterController` to bypass estimation during early development and debugging.

---

## Published topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 200 Hz | Full base pose + velocity with Kalman covariance |
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 200 Hz | Base pose only |
