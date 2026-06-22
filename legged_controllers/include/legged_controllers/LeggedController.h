//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <atomic>
#include <thread>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_estimation/LinearKalmanFilter.h>

#include <legged_interface/LeggedInterface.h>

namespace legged {

class LeggedController : public controller_interface::ControllerInterface
{
public:
  LeggedController() = default;
  ~LeggedController() override;


  // overided methods from ControllerInterface
  // must be implemented
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // ── Setup functions called from on_configure() ───────────────────────────
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                    const std::string& referenceFile, bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // ── Per-cycle helpers called from update() ───────────────────────────────
  virtual void updateStatesEstimatate(const rclcpp::Time& time, const rclcpp::Duration& period);

  

  // ── Parameters set in on_init() ─────────────────────────────────────────
  std::string urdfFile_;
  std::string taskFile_;
  std::string referenceFile_;
  std::vector<std::string> jointNames_;
  std::string imuName_;

  // ── OCS2 legged interface (optimal control problem + model) ──────────────
  std::shared_ptr<LeggedInterface> legged_interface_;

  // ── MPC solver (SQP) + MRT bridge ───────────────────────────────────────
  std::shared_ptr<ocs2::MPC_BASE> mpc_;
  std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;

  // Converts between MPC centroidal state and full rigid-body state.
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;

  // Plain rclcpp::Node used for all OCS2 ROS pub/sub (GaitReceiver,
  // RosReferenceManager, observation publisher). Needed because get_node()
  // returns LifecycleNode::SharedPtr which is incompatible with Node::SharedPtr.
  rclcpp::Node::SharedPtr mpcNode_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;

  // ── State estimation ─────────────────────────────────────────────────────
  std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  // Full rigid-body state from the estimator; converted to centroidal for MPC.
  ocs2::vector_t measuredRbdState_;
  // Current MPC observation: mode, time, centroidal state + input.
  SystemObservation currentObservation_;

  // Standing joint angles loaded from reference.info defaultJointState.
  // Used as the PD position setpoint so the robot holds the nominal standing pose.
  ocs2::vector_t defaultJointAngles_;

  // ── MPC background thread ────────────────────────────────────────────────
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{false};
  std::atomic_bool mpcRunning_{false};

  // ── Benchmarking ─────────────────────────────────────────────────────────
  ocs2::benchmark::RepeatedTimer mpcTimer_;
  ocs2::benchmark::RepeatedTimer wbcTimer_;

};

}  // namespace legged
