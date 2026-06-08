#pragma once

#include <cstdint>

namespace udp_interface {

static constexpr int NUM_JOINTS = 12;

// Sent by this node to the controller (MuJoCo → controller).
// Contains a full robot state snapshot: joints + IMU.
struct alignas(8) RobotStatePacket
{
  uint64_t timestamp_ns;          // host clock, nanoseconds

  // Joints — canonical order matches joint_names parameter
  float joint_pos[NUM_JOINTS];    // rad
  float joint_vel[NUM_JOINTS];    // rad/s
  float joint_torque[NUM_JOINTS]; // Nm  (effort from /joint_states)

  // IMU
  float imu_quat[4];   // w, x, y, z
  float imu_gyro[3];   // rad/s  (angular velocity)
  float imu_accel[3];  // m/s²   (linear acceleration)
};

// Sent by the controller to this node (controller → MuJoCo).
// PD + feedforward torque command per joint.
struct alignas(8) JointCommandPacket
{
  uint64_t timestamp_ns;         // controller clock, nanoseconds

  float pos_des[NUM_JOINTS];     // rad      — desired position
  float vel_des[NUM_JOINTS];     // rad/s    — desired velocity
  float torque_ff[NUM_JOINTS];   // Nm       — feedforward torque
  float kp[NUM_JOINTS];          // Nm/rad   — position gain
  float kd[NUM_JOINTS];          // Nm·s/rad — velocity gain
};

}  // namespace udp_interface
