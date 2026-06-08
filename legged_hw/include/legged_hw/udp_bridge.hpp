#pragma once

// UDP packet layout and socket helper.
// Packet structs MUST stay byte-identical to mujoco_ros2_control/udp_bridge.hpp.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>

namespace legged_hw {

static constexpr uint32_t UDP_MAX_JOINTS = 32;

// Sent by MujocoSystemInterface → received here (sim/robot → legged_hw).
struct alignas(8) RobotStatePacket
{
  uint64_t timestamp_ns;
  uint32_t num_joints;
  uint32_t _pad{};

  float joint_pos   [UDP_MAX_JOINTS];   // rad
  float joint_vel   [UDP_MAX_JOINTS];   // rad/s
  float joint_torque[UDP_MAX_JOINTS];   // Nm

  float imu_quat[4];   // w, x, y, z
  float imu_gyro[3];   // rad/s
  float imu_accel[3];  // m/s²
};

// Sent here → received by MujocoSystemInterface (legged_hw → sim/robot).
struct alignas(8) JointCommandPacket
{
  uint64_t timestamp_ns;
  uint32_t num_joints;
  uint32_t _pad{};

  float pos_des  [UDP_MAX_JOINTS];   // rad
  float vel_des  [UDP_MAX_JOINTS];   // rad/s
  float torque_ff[UDP_MAX_JOINTS];   // Nm
  float kp       [UDP_MAX_JOINTS];   // Nm/rad
  float kd       [UDP_MAX_JOINTS];   // Nm·s/rad
};

// Minimal POSIX UDP socket helper (no external deps).
class UdpSocket
{
public:
  ~UdpSocket() { close(); }

  bool open_sender(const std::string& remote_ip, uint16_t remote_port)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;
    std::memset(&remote_, 0, sizeof(remote_));
    remote_.sin_family = AF_INET;
    remote_.sin_port   = htons(remote_port);
    return ::inet_pton(AF_INET, remote_ip.c_str(), &remote_.sin_addr) == 1;
  }

  bool open_receiver(uint16_t local_port, int recv_timeout_ms = 100)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;
    int opt = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (recv_timeout_ms > 0) {
      timeval tv{recv_timeout_ms / 1000, (recv_timeout_ms % 1000) * 1000};
      ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(local_port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      close(); return false;
    }
    return true;
  }

  bool send(const void* data, size_t len)
  {
    return ::sendto(fd_, data, len, 0,
                    reinterpret_cast<const sockaddr*>(&remote_), sizeof(remote_))
           == static_cast<ssize_t>(len);
  }

  ssize_t recv(void* buf, size_t max_len)
  {
    return ::recvfrom(fd_, buf, max_len, 0, nullptr, nullptr);
  }

  bool is_open() const { return fd_ >= 0; }

  void close()
  {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

private:
  int         fd_{-1};
  sockaddr_in remote_{};
};

}  // namespace legged_hw
