#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "udp_interface/UdpSocket.hpp"
#include "udp_interface/packets.hpp"

using namespace std::chrono_literals;

namespace udp_interface {

class UdpBridgeNode : public rclcpp::Node
{
public:
  UdpBridgeNode() : Node("udp_bridge_node")
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    declare_parameter("controller_ip",     std::string("127.0.0.1"));
    declare_parameter("state_port",        9870);  // node sends state TO this port
    declare_parameter("cmd_port",          9871);  // node listens for commands ON this port
    declare_parameter("joint_names",       std::vector<std::string>{
      "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
      "left_knee_pitch_joint", "left_ankle_roll_joint","left_ankle_pitch_joint",
      "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
      "right_knee_pitch_joint","right_ankle_roll_joint","right_ankle_pitch_joint"
    });
    declare_parameter("joint_states_topic", std::string("/joint_states"));
    declare_parameter("imu_topic",          std::string("/imu_sensor_broadcaster/imu"));

    const auto controller_ip = get_parameter("controller_ip").as_string();
    const auto state_port    = static_cast<uint16_t>(get_parameter("state_port").as_int());
    const auto cmd_port      = static_cast<uint16_t>(get_parameter("cmd_port").as_int());
    joint_names_             = get_parameter("joint_names").as_string_array();
    const auto js_topic      = get_parameter("joint_states_topic").as_string();
    const auto imu_topic     = get_parameter("imu_topic").as_string();

    // ── UDP sockets ──────────────────────────────────────────────────────────
    if (!send_socket_.open_sender(controller_ip, state_port)) {
      RCLCPP_FATAL(get_logger(), "Failed to open UDP send socket to %s:%u",
                   controller_ip.c_str(), state_port);
      throw std::runtime_error("UDP send socket failed");
    }
    if (!recv_socket_.open_receiver(cmd_port)) {
      RCLCPP_FATAL(get_logger(), "Failed to bind UDP recv socket on port %u", cmd_port);
      throw std::runtime_error("UDP recv socket failed");
    }

    RCLCPP_INFO(get_logger(),
                "UDP bridge ready — sending state to %s:%u, receiving commands on :%u",
                controller_ip.c_str(), state_port, cmd_port);

    // ── Subscribers ──────────────────────────────────────────────────────────
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      js_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(msg); });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(msg); });

    // ── Publisher — received UDP commands re-published as ROS2 topic ─────────
    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/udp_joint_commands",
                                                               rclcpp::SensorDataQoS());

    // ── Background recv thread ───────────────────────────────────────────────
    recv_running_ = true;
    recv_thread_  = std::thread([this]() { recvLoop(); });
  }

  ~UdpBridgeNode() override
  {
    recv_running_ = false;
    recv_socket_.close();
    if (recv_thread_.joinable()) recv_thread_.join();
  }

private:
  // ── Callbacks ──────────────────────────────────────────────────────────────

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr& msg)
  {
    RobotStatePacket pkt{};
    pkt.timestamp_ns = static_cast<uint64_t>(
      rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());

    // Map named joints → packet slots by canonical order
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) continue;
      const size_t idx = static_cast<size_t>(it - msg->name.begin());

      pkt.joint_pos[i]    = idx < msg->position.size() ?
                              static_cast<float>(msg->position[idx]) : 0.f;
      pkt.joint_vel[i]    = idx < msg->velocity.size() ?
                              static_cast<float>(msg->velocity[idx]) : 0.f;
      pkt.joint_torque[i] = idx < msg->effort.size() ?
                              static_cast<float>(msg->effort[idx])   : 0.f;
    }

    // Copy latest IMU snapshot
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      pkt.imu_quat[0] = cached_imu_.imu_quat[0];
      pkt.imu_quat[1] = cached_imu_.imu_quat[1];
      pkt.imu_quat[2] = cached_imu_.imu_quat[2];
      pkt.imu_quat[3] = cached_imu_.imu_quat[3];
      pkt.imu_gyro[0] = cached_imu_.imu_gyro[0];
      pkt.imu_gyro[1] = cached_imu_.imu_gyro[1];
      pkt.imu_gyro[2] = cached_imu_.imu_gyro[2];
      pkt.imu_accel[0] = cached_imu_.imu_accel[0];
      pkt.imu_accel[1] = cached_imu_.imu_accel[1];
      pkt.imu_accel[2] = cached_imu_.imu_accel[2];
    }

    send_socket_.send(&pkt, sizeof(pkt));
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    cached_imu_.imu_quat[0] = static_cast<float>(msg->orientation.w);
    cached_imu_.imu_quat[1] = static_cast<float>(msg->orientation.x);
    cached_imu_.imu_quat[2] = static_cast<float>(msg->orientation.y);
    cached_imu_.imu_quat[3] = static_cast<float>(msg->orientation.z);
    cached_imu_.imu_gyro[0] = static_cast<float>(msg->angular_velocity.x);
    cached_imu_.imu_gyro[1] = static_cast<float>(msg->angular_velocity.y);
    cached_imu_.imu_gyro[2] = static_cast<float>(msg->angular_velocity.z);
    cached_imu_.imu_accel[0] = static_cast<float>(msg->linear_acceleration.x);
    cached_imu_.imu_accel[1] = static_cast<float>(msg->linear_acceleration.y);
    cached_imu_.imu_accel[2] = static_cast<float>(msg->linear_acceleration.z);
  }

  // ── UDP receive loop (background thread) ───────────────────────────────────

  void recvLoop()
  {
    JointCommandPacket pkt{};
    while (recv_running_) {
      const ssize_t n = recv_socket_.recv(&pkt, sizeof(pkt));
      if (n != static_cast<ssize_t>(sizeof(pkt))) continue;  // timeout or bad size

      // Republish on ROS2 topic so the controller can consume it
      auto ros_msg = sensor_msgs::msg::JointState();
      ros_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      ros_msg.name.reserve(NUM_JOINTS);
      ros_msg.position.reserve(NUM_JOINTS);
      ros_msg.velocity.reserve(NUM_JOINTS);
      ros_msg.effort.reserve(NUM_JOINTS);

      for (int i = 0; i < NUM_JOINTS; ++i) {
        ros_msg.name.push_back(joint_names_[i]);
        ros_msg.position.push_back(pkt.pos_des[i]);
        ros_msg.velocity.push_back(pkt.vel_des[i]);
        ros_msg.effort.push_back(pkt.torque_ff[i]);
      }
      cmd_pub_->publish(ros_msg);
    }
  }

  // ── Members ────────────────────────────────────────────────────────────────

  std::vector<std::string> joint_names_;

  UdpSocket send_socket_;
  UdpSocket recv_socket_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     cmd_pub_;

  // Latest IMU snapshot (written by imu callback, read by joint-state callback)
  struct ImuCache { float imu_quat[4]{}; float imu_gyro[3]{}; float imu_accel[3]{}; };
  ImuCache  cached_imu_{};
  std::mutex imu_mutex_;

  std::thread       recv_thread_;
  std::atomic<bool> recv_running_{false};
};

}  // namespace udp_interface

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<udp_interface::UdpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
