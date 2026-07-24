// Subscribes to /joint_states, runs Pinocchio forward kinematics, and opens a
// MuJoCo window showing the robot with:
//   - joint angles driven from /joint_states  (MuJoCo's own FK moves the meshes)
//   - green sphere  = left  foot position computed by Pinocchio
//   - orange sphere = right foot position computed by Pinocchio
//
// If MuJoCo FK and Pinocchio FK agree the spheres sit exactly inside the foot meshes.
//
// Also publishes:
//   /pinocchio_fk/<contact>_pose   geometry_msgs/PoseStamped  (in torso frame)
// and broadcasts TF:  link0_torso -> <contact>_fk
//
// Parameters:
//   urdf_file           path to biped_robot_12dof.urdf
//   task_file           path to task.info
//   mujoco_model_path   path to fixed_torso_test.xml  (default)
//   joint_states_topic  (default /joint_states)
//   base_z              fixed base height in metres (default 0.765)

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "robot_state_visualization/mujoco_rendering.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <Eigen/Geometry>
#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ocs2;

// ---------------------------------------------------------------------------
class ForwardKinematicsNode : public rclcpp::Node
{
public:
  // Pinocchio-computed foot positions in world frame (filled by compute())
  std::array<Eigen::Vector3d, 2> footWorldPos{};

  ForwardKinematicsNode() : rclcpp::Node("forward_kinematics_node")
  {
    declare_parameter<std::string>("urdf_file", "");
    declare_parameter<std::string>("task_file", "");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<double>("base_z", 0.765);
    declare_parameter<std::string>("mujoco_model_path", "");
  }

  void initialize()
  {
    const auto urdfFile = get_parameter("urdf_file").as_string();
    const auto taskFile = get_parameter("task_file").as_string();
    const auto jsTopic  = get_parameter("joint_states_topic").as_string();
    baseZ_              = get_parameter("base_z").as_double();

    if (urdfFile.empty() || taskFile.empty()) {
      RCLCPP_FATAL(get_logger(), "urdf_file and task_file parameters are required.");
      throw std::runtime_error("Missing required parameters.");
    }

    // ── Load joint/contact names from task.info ───────────────────────────────
    loadData::loadStdVector(taskFile, "model_settings.jointNames",      jointNames_,   false);
    loadData::loadStdVector(taskFile, "model_settings.contactNames3DoF", contactNames_, false);
    RCLCPP_INFO(get_logger(), "Joints: %zu, contacts: %zu", jointNames_.size(), contactNames_.size());

    // ── Pinocchio interface ───────────────────────────────────────────────────
    pinInterface_ = std::make_unique<PinocchioInterface>(
        centroidal_model::createPinocchioInterface(urdfFile, jointNames_));

    const auto& model = pinInterface_->getModel();
    nJoints_ = static_cast<size_t>(model.nq - 7);

    torsoFrameId_ = model.getFrameId("link0_torso");
    for (const auto& name : contactNames_) {
      const auto id = model.getFrameId(name);
      if (id < static_cast<pinocchio::FrameIndex>(model.nframes))
        contactFrameIds_.push_back(id);
      else
        RCLCPP_WARN(get_logger(), "Frame '%s' not found.", name.c_str());
    }

    RCLCPP_INFO(get_logger(), "Pinocchio: nq=%d, torso_frame=%zu, contacts=%zu",
                model.nq, torsoFrameId_, contactFrameIds_.size());

    // ── ROS publishers ────────────────────────────────────────────────────────
    for (const auto& name : contactNames_) {
      footPosePubs_.push_back(
          create_publisher<geometry_msgs::msg::PoseStamped>(
              "/pinocchio_fk/" + name + "_pose", rclcpp::SystemDefaultsQoS()));
    }
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── Joint state subscription ──────────────────────────────────────────────
    jointStateSub_ = create_subscription<sensor_msgs::msg::JointState>(
        jsTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          latestJs_ = std::move(msg);
        });

    RCLCPP_INFO(get_logger(), "Ready. Subscribed to '%s'.", jsTopic.c_str());
  }

  // Called from the main render loop every frame.
  // Returns false if no joint state received yet.
  bool compute()
  {
    sensor_msgs::msg::JointState::SharedPtr js;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      js = latestJs_;
    }
    if (!js) return false;

    const auto& model = pinInterface_->getModel();
    auto& data = pinInterface_->getData();

    // ── Build Pinocchio q ─────────────────────────────────────────────────────
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    q(2) = baseZ_;
    q(6) = 1.0;  // identity quaternion w

    for (size_t i = 0; i < nJoints_; ++i) {
      const auto it = std::find(js->name.begin(), js->name.end(), jointNames_[i]);
      if (it == js->name.end()) return false;
      const size_t idx = static_cast<size_t>(std::distance(js->name.begin(), it));
      if (idx < js->position.size()) q(7 + i) = js->position[idx];
    }
    latestQ_ = q;  // expose for MuJoCo joint sync

    // ── Pinocchio FK ──────────────────────────────────────────────────────────
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const pinocchio::SE3& T_world_torso = data.oMf[torsoFrameId_];
    const rclcpp::Time stamp = now();

    for (size_t i = 0; i < contactFrameIds_.size(); ++i) {
      const pinocchio::SE3& T_world_foot = data.oMf[contactFrameIds_[i]];
      footWorldPos[i] = T_world_foot.translation();  // for MuJoCo site marker

      const pinocchio::SE3  T_torso_foot = T_world_torso.inverse() * T_world_foot;
      const Eigen::Vector3d    pos  = T_torso_foot.translation();
      const Eigen::Quaterniond quat(T_torso_foot.rotation());

      geometry_msgs::msg::PoseStamped poseMsg;
      poseMsg.header.stamp    = stamp;
      poseMsg.header.frame_id = "link0_torso";
      poseMsg.pose.position.x = pos.x();
      poseMsg.pose.position.y = pos.y();
      poseMsg.pose.position.z = pos.z();
      poseMsg.pose.orientation.x = quat.x();
      poseMsg.pose.orientation.y = quat.y();
      poseMsg.pose.orientation.z = quat.z();
      poseMsg.pose.orientation.w = quat.w();
      footPosePubs_[i]->publish(poseMsg);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp    = stamp;
      tf.header.frame_id = "link0_torso";
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
    return true;
  }

  // Latest Pinocchio q vector (joints + base), for syncing into MuJoCo qpos
  Eigen::VectorXd latestQ_;

  const std::vector<std::string>& jointNames() const { return jointNames_; }

private:
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

  std::mutex mutex_;
  sensor_msgs::msg::JointState::SharedPtr latestJs_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardKinematicsNode>();
  node->initialize();

  // ── MuJoCo model ──────────────────────────────────────────────────────────
  const auto mjcfPath = node->get_parameter("mujoco_model_path").as_string();
  if (mjcfPath.empty()) {
    RCLCPP_FATAL(node->get_logger(), "mujoco_model_path parameter is required.");
    return 1;
  }

  char error[1000] = "";
  mjModel* mj_model = mj_loadXML(mjcfPath.c_str(), nullptr, error, sizeof(error));
  if (!mj_model) {
    RCLCPP_FATAL(node->get_logger(), "MuJoCo load error: %s", error);
    return 1;
  }
  mjData* mj_data = mj_makeData(mj_model);
  RCLCPP_INFO(node->get_logger(), "MuJoCo model loaded: %s", mjcfPath.c_str());

  // ── Build joint name → qpos address map ──────────────────────────────────
  std::unordered_map<std::string, int> jointQposMap;
  for (int j = 0; j < mj_model->njnt; ++j) {
    const std::string name = mj_model->names + mj_model->name_jntadr[j];
    jointQposMap[name] = mj_model->jnt_qposadr[j];
  }

  // ── Find Pinocchio marker site IDs ────────────────────────────────────────
  const int leftSiteId  = mj_name2id(mj_model, mjOBJ_SITE, "left_foot_pinocchio");
  const int rightSiteId = mj_name2id(mj_model, mjOBJ_SITE, "right_foot_pinocchio");
  if (leftSiteId < 0 || rightSiteId < 0) {
    RCLCPP_WARN(node->get_logger(),
                "Marker sites not found in MJCF (left=%d, right=%d). "
                "Add left_foot_pinocchio / right_foot_pinocchio sites to worldbody.",
                leftSiteId, rightSiteId);
  }

  // ── GLFW + MuJoCo rendering ───────────────────────────────────────────────
  if (!glfwInit()) {
    RCLCPP_FATAL(node->get_logger(), "GLFW init failed.");
    return 1;
  }
  auto* rendering = robot_state_visualization::MujocoRendering::get_instance();
  rendering->init(mj_model, mj_data);
  RCLCPP_INFO(node->get_logger(), "Rendering window open.");

  // ── Render loop (GLFW must stay on main thread) ───────────────────────────
  constexpr double kRenderHz = 60.0;
  constexpr double kRenderDt = 1.0 / kRenderHz;
  auto lastRender = std::chrono::high_resolution_clock::now();

  while (rclcpp::ok() && !rendering->is_close_flag_raised()) {
    // Process ROS callbacks (joint states subscription)
    rclcpp::spin_some(node);

    // Run Pinocchio FK + publish topics
    const bool hasData = node->compute();

    if (hasData && !node->latestQ_.size() == 0) {
      // ── Sync joint angles into MuJoCo qpos ─────────────────────────────────
      for (const auto& jname : node->jointNames()) {
        auto it = jointQposMap.find(jname);
        if (it == jointQposMap.end()) continue;
        const size_t pinIdx = static_cast<size_t>(
            std::find(node->jointNames().begin(), node->jointNames().end(), jname)
            - node->jointNames().begin());
        if (7 + pinIdx < static_cast<size_t>(node->latestQ_.size()))
          mj_data->qpos[it->second] = node->latestQ_(7 + pinIdx);
      }

      // ── Advance MuJoCo kinematics (no physics step) ───────────────────────
      mj_forward(mj_model, mj_data);

      // ── Override site positions with Pinocchio world-frame foot positions ──
      // This runs AFTER mj_forward so MuJoCo doesn't overwrite them.
      if (leftSiteId >= 0) {
        const auto& p = node->footWorldPos[0];
        mj_data->site_xpos[3 * leftSiteId + 0] = p.x();
        mj_data->site_xpos[3 * leftSiteId + 1] = p.y();
        mj_data->site_xpos[3 * leftSiteId + 2] = p.z();
      }
      if (rightSiteId >= 0) {
        const auto& p = node->footWorldPos[1];
        mj_data->site_xpos[3 * rightSiteId + 0] = p.x();
        mj_data->site_xpos[3 * rightSiteId + 1] = p.y();
        mj_data->site_xpos[3 * rightSiteId + 2] = p.z();
      }
    }

    // ── Render at 60 Hz ───────────────────────────────────────────────────────
    const auto now = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration<double>(now - lastRender).count() >= kRenderDt) {
      rendering->update();
      lastRender = now;
    }
  }

  rendering->close();
  mj_deleteData(mj_data);
  mj_deleteModel(mj_model);
  rclcpp::shutdown();
  return 0;
}
