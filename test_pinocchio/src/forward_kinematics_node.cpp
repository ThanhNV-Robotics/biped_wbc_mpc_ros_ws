// Subscribes to /joint_states from MuJoCo, runs Pinocchio forward kinematics,
// and publishes the computed foot poses in the pelvis (torso) frame.
//
// Assumes fixed base at origin (no IMU / odometry required).
// Run alongside the MuJoCo simulation and compare published poses to the
// visual foot positions in the renderer.
//
// Topics published:
//   /pinocchio_fk/left_foot_pose   geometry_msgs/PoseStamped  (in torso frame)
//   /pinocchio_fk/right_foot_pose  geometry_msgs/PoseStamped  (in torso frame)
//
// TF broadcast:
//   pelvis -> left_foot_fk
//   pelvis -> right_foot_fk
//
// Usage (example):
//   ros2 run test_pinocchio forward_kinematics_node \
//     --ros-args \
//     -p urdf_file:=$(ros2 pkg prefix biped_description)/share/biped_description/urdf/biped_robot_12dof.urdf \
//     -p task_file:=$(ros2 pkg prefix legged_controllers)/share/legged_controllers/config/mpc/task.info \
//     -p base_z:=0.765

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>

using namespace ocs2;

// ---------------------------------------------------------------------------
class ForwardKinematicsNode : public rclcpp::Node
{
public:
  ForwardKinematicsNode() : rclcpp::Node("forward_kinematics_node")
  {
    declare_parameter<std::string>("urdf_file", "");
    declare_parameter<std::string>("task_file", "");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<double>("base_z", 0.765);
    declare_parameter<double>("update_rate", 100.0);
  }

  void initialize()
  {
    const auto urdfFile   = get_parameter("urdf_file").as_string();
    const auto taskFile   = get_parameter("task_file").as_string();
    const auto jsTopic    = get_parameter("joint_states_topic").as_string();
    baseZ_                = get_parameter("base_z").as_double();
    const double rate     = get_parameter("update_rate").as_double();

    if (urdfFile.empty() || taskFile.empty()) {
      RCLCPP_FATAL(get_logger(), "urdf_file and task_file parameters are required.");
      throw std::runtime_error("Missing required parameters.");
    }

    // ── Load joint names from task.info ──────────────────────────────────────
    loadData::loadStdVector(taskFile, "model_settings.jointNames", jointNames_, false);
    loadData::loadStdVector(taskFile, "model_settings.contactNames3DoF", contactNames_, false);
    RCLCPP_INFO(get_logger(), "Joints: %zu, 3-DoF contacts: %zu",
                jointNames_.size(), contactNames_.size());

    // ── Build Pinocchio interface ─────────────────────────────────────────────
    pinInterface_ = std::make_unique<PinocchioInterface>(
        centroidal_model::createPinocchioInterface(urdfFile, jointNames_));

    const auto& model = pinInterface_->getModel();
    nJoints_ = static_cast<size_t>(model.nq - 7);  // subtract 3 pos + 4 quat for free-flyer

    RCLCPP_INFO(get_logger(), "Pinocchio model: nq=%d, nv=%d, nJoints=%zu",
                model.nq, model.nv, nJoints_);

    // ── Resolve frame IDs ─────────────────────────────────────────────────────
    torsoFrameId_ = model.getFrameId("link0_torso");
    RCLCPP_INFO(get_logger(), "Torso frame id: %zu", torsoFrameId_);

    for (const auto& name : contactNames_) {
      const auto id = model.getFrameId(name);
      if (id == static_cast<pinocchio::FrameIndex>(model.nframes)) {
        RCLCPP_WARN(get_logger(), "Frame '%s' not found in model — skipping.", name.c_str());
      } else {
        contactFrameIds_.push_back(id);
        RCLCPP_INFO(get_logger(), "Contact frame '%s' -> id %zu", name.c_str(), id);
      }
    }

    // ── Publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::SystemDefaultsQoS();
    for (const auto& name : contactNames_) {
      const std::string topic = "/pinocchio_fk/" + name + "_pose";
      footPosePubs_.push_back(
          create_publisher<geometry_msgs::msg::PoseStamped>(topic, qos));
      RCLCPP_INFO(get_logger(), "Publishing FK to: %s", topic.c_str());
    }

    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── Subscription ─────────────────────────────────────────────────────────
    jointStateSub_ = create_subscription<sensor_msgs::msg::JointState>(
        jsTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latestJs_ = std::move(msg);
        });

    // ── Timer ─────────────────────────────────────────────────────────────────
    const auto periodNs = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate));
    timer_ = create_wall_timer(periodNs, [this]() { compute(); });

    RCLCPP_INFO(get_logger(), "Ready. Subscribed to '%s', publishing at %.0f Hz.",
                jsTopic.c_str(), rate);
  }

private:
  void compute()
  {
    sensor_msgs::msg::JointState::SharedPtr js;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      js = latestJs_;
    }
    if (!js) return;

    const auto& model = pinInterface_->getModel();
    auto& data  = pinInterface_->getData();

    // ── Build Pinocchio q: [pos(3), quat_xyzw(4), joints(nJ)] ───────────────
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    // Base fixed at world origin at standing height; identity orientation
    q(2) = baseZ_;
    q(6) = 1.0;  // quaternion w = 1 (identity)

    for (size_t i = 0; i < nJoints_; ++i) {
      const auto it = std::find(js->name.begin(), js->name.end(), jointNames_[i]);
      if (it == js->name.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Joint '%s' not in /joint_states", jointNames_[i].c_str());
        return;
      }
      const size_t idx = static_cast<size_t>(std::distance(js->name.begin(), it));
      if (idx < js->position.size()) {
        q(7 + i) = js->position[idx];
      }
    }

    // ── Forward kinematics ───────────────────────────────────────────────────
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // ── Torso pose in world ───────────────────────────────────────────────────
    const pinocchio::SE3& T_world_torso = data.oMf[torsoFrameId_];

    const rclcpp::Time stamp = now();
    const std::string torsoFrame = "link0_torso";

    // ── Publish each contact foot pose relative to torso ─────────────────────
    for (size_t i = 0; i < contactFrameIds_.size(); ++i) {
      const pinocchio::SE3& T_world_foot = data.oMf[contactFrameIds_[i]];
      const pinocchio::SE3  T_torso_foot = T_world_torso.inverse() * T_world_foot;

      const Eigen::Vector3d    pos = T_torso_foot.translation();
      const Eigen::Quaterniond quat(T_torso_foot.rotation());

      // PoseStamped
      geometry_msgs::msg::PoseStamped poseMsg;
      poseMsg.header.stamp    = stamp;
      poseMsg.header.frame_id = torsoFrame;
      poseMsg.pose.position.x = pos.x();
      poseMsg.pose.position.y = pos.y();
      poseMsg.pose.position.z = pos.z();
      poseMsg.pose.orientation.x = quat.x();
      poseMsg.pose.orientation.y = quat.y();
      poseMsg.pose.orientation.z = quat.z();
      poseMsg.pose.orientation.w = quat.w();
      footPosePubs_[i]->publish(poseMsg);

      // TF: torso -> <contact_name>_fk
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = stamp;
      tf.header.frame_id = torsoFrame;
      tf.child_frame_id  = contactNames_[i] + "_fk";
      tf.transform.translation.x = pos.x();
      tf.transform.translation.y = pos.y();
      tf.transform.translation.z = pos.z();
      tf.transform.rotation.x = quat.x();
      tf.transform.rotation.y = quat.y();
      tf.transform.rotation.z = quat.z();
      tf.transform.rotation.w = quat.w();
      tfBroadcaster_->sendTransform(tf);
    }

    RCLCPP_DEBUG(get_logger(),
                 "FK: left_foot=[%.3f, %.3f, %.3f]  right_foot=[%.3f, %.3f, %.3f]",
                 data.oMf[contactFrameIds_[0]].translation().x() - T_world_torso.translation().x(),
                 data.oMf[contactFrameIds_[0]].translation().y() - T_world_torso.translation().y(),
                 data.oMf[contactFrameIds_[0]].translation().z() - T_world_torso.translation().z(),
                 data.oMf[contactFrameIds_[1]].translation().x() - T_world_torso.translation().x(),
                 data.oMf[contactFrameIds_[1]].translation().y() - T_world_torso.translation().y(),
                 data.oMf[contactFrameIds_[1]].translation().z() - T_world_torso.translation().z());
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::unique_ptr<PinocchioInterface> pinInterface_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> contactNames_;
  std::vector<pinocchio::FrameIndex> contactFrameIds_;
  pinocchio::FrameIndex torsoFrameId_{0};
  size_t nJoints_{0};
  double baseZ_{0.765};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> footPosePubs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  sensor_msgs::msg::JointState::SharedPtr latestJs_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardKinematicsNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
