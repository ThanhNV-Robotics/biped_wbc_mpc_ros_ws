//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace legged {
using namespace ocs2;

// This class inherits from StateEstimateBase and implements a linear Kalman filter for state estimation. It also subscribes to an odometry topic to get measurements for the filter update. The estimated state is published as an odometry message.
class KalmanFilterEstimate : public StateEstimateBase {
 public:
  KalmanFilterEstimate(rclcpp::Node::SharedPtr node,
                       PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                       const PinocchioEndEffectorKinematics& eeKinematics);

  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();

  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  nav_msgs::msg::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

 private:
  size_t numContacts_, dimContacts_, numState_, numObserve_;

  matrix_t a_, b_, c_, q_, p_, r_;
  vector_t xHat_, ps_, vs_;
  
  // Intialization

  bool biasInitialized_ = false;

  double biasInitDuration_ = 10.0; // seconds
  Eigen::Vector3d positionBias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d positionBiasSum_ = Eigen::Vector3d::Zero();

  size_t biasSampleCount_ = 0;

  rclcpp::Time biasInitStartTime_;
  double elapsedTime_ = 0.0;



  // Topic
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
  bool initialized_ = false;
};

}  // namespace legged
