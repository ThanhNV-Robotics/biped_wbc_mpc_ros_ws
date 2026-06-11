# Gazebo Hardware Interface — Data Communication

`LeggedHWSim` uses **no ROS topics** between Gazebo and the controller. Everything is direct C++ API calls into the Gazebo physics engine. Both run in the same process, so data is shared via pointers — the same pattern as the real hardware.

---

## Initialization (`initSim`)

At startup, `LeggedHWSim` acquires direct pointers into the Gazebo physics engine and registers hardware interface handles pointing to local data buffers:

- `sim_joints_[]` — array of `gazebo::physics::Joint*` pointers (from `DefaultRobotHWSim`)
- `imu.linkPtr_` — `gazebo::physics::LinkPtr` to the robot body link (from `parseImu`)
- `contactManager_` — `gazebo::physics::ContactManager*` to the world's contact manager

The hardware interface handles (`HybridJointHandle`, `ImuSensorHandle`, `ContactSensorHandle`) are registered with pointers into local buffers (`HybridJointData`, `ImuData`, `name2contact_`). The controller holds copies of these handles — same pointers, shared memory.

---

## Reading Sensor Data (`readSim`, called every 1ms)

### Joint States ([LeggedHWSim.cpp:79–91](legged_gazebo/src/LeggedHWSim.cpp#L79))

Direct calls into Gazebo's joint objects:

```cpp
double position  = sim_joints_[j]->Position(0);
joint_velocity_[j] = (position - joint_position_[j]) / period.toSec();  // finite difference
joint_effort_[j]   = sim_joints_[j]->GetForce(0);
```

Note: velocity is computed by finite differencing position rather than using Gazebo's built-in velocity (which has a known bias in `DefaultRobotHWSim`).

### IMU ([LeggedHWSim.cpp:95–112](legged_gazebo/src/LeggedHWSim.cpp#L95))

Direct calls on the body `LinkPtr`:

```cpp
ignition::math::Pose3d pose  = imu.linkPtr_->WorldPose();
ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel()
                                 - pose.Rot().RotateVectorReverse(gravity);  // subtract gravity
```

Results are written into `ImuData.ori_[]`, `angularVel_[]`, `linearAcc_[]` — the same arrays the `ImuSensorHandle` points to, so the controller sees them immediately.

### Foot Contacts ([LeggedHWSim.cpp:115–131](legged_gazebo/src/LeggedHWSim.cpp#L115))

The global `ContactManager` is polled each cycle. Any contact whose timestamp matches the current simulation step is checked against the registered foot link names:

```cpp
for (const auto& contact : contactManager_->GetContacts()) {
    // filter by timestamp to current sim step only
    std::string linkName = contact->collision1->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end())
        name2contact_[linkName] = true;
}
```

`name2contact_[foot]` is a `bool` that the `ContactSensorHandle` points to directly.

---

## Writing Commands (`writeSim`, called every 1ms)

The controller writes `posDes, velDes, kp, kd, ff` into the `HybridJointData` buffers via `setCommand()`. `writeSim` then:

1. **Buffers the command** into a timestamped `deque` per joint (to simulate actuator delay, configurable via `gazebo/delay` param)
2. **Evicts stale entries** older than `delay_` seconds
3. **Evaluates the PD law** on the oldest buffered command and applies the resulting torque to Gazebo:

```cpp
joint.joint_.setCommand(
    cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) +
    cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
    cmd.ff_
);
```

This torque is applied to the Gazebo joint in the next physics step via `DefaultRobotHWSim::writeSim()`.

---

## Full Data Flow

```
Gazebo physics engine (same process, no topics)
  │
  ├─ sim_joints_[j]->Position() / GetForce()
  │     └─► joint_position_[], joint_velocity_[], joint_effort_[]
  │              └─► JointStateInterface handles (pointers)
  │
  ├─ imu.linkPtr_->WorldPose() / RelativeAngularVel() / RelativeLinearAccel()
  │     └─► ImuData.ori_[], angularVel_[], linearAcc_[]
  │              └─► ImuSensorInterface handles (pointers)
  │
  └─ contactManager_->GetContacts()
        └─► name2contact_[foot] bool
                 └─► ContactSensorInterface handles (pointers)
                          │
                          ▼
              Controller reads state each cycle
                          │
                          ▼
              Controller writes setCommand(posDes, velDes, kp, kd, ff)
                          │
                          ▼
              writeSim: delay buffer → PD law → joint_.setCommand(torque)
                          │
                          ▼
              Gazebo physics applies torque in next step
```

---

## Key Points

- **No ROS topics** — all communication is in-process via C++ pointers and direct Gazebo API calls.
- **Velocity is finite-differenced** — not taken directly from Gazebo, to avoid a bias in `DefaultRobotHWSim`.
- **Gravity is subtracted from IMU** — `RelativeLinearAccel` includes gravity; it is removed to match real IMU behavior.
- **Delay buffer** — `writeSim` can simulate actuator latency (`gazebo/delay` param), making sim behavior closer to real hardware.
- **Contact timestamp filtering** — only contacts from the immediately preceding physics step are used, avoiding stale detections.
