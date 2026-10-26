#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "semantic_components/imu_sensor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


#include "biped_wbc/state_estimation/contact_fusion.hpp"
#include "biped_wbc/state_estimation/StateEstimator.hpp"
#include <tsid/robots/robot-wrapper.hpp>
#include "biped_wbc/QP_tsid/QPtsid.hpp"
#include "biped_wbc/planner/LIPTaskPlanner.hpp"

#include "wbc_controller/controller_params.hpp"
#include "wbc_controller/joint_map.hpp"
#include "wbc_controller/robot_interface.hpp"



namespace biped_controllers
{


class WBCController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Steps of the lifecycle, virtual so that a variant of the controller can replace one of them
  // (e.g. a "cheater" controller whose setupStateEstimate() / estimate() use the simulator's
  // ground truth, as LeggedCheaterController in legged_control).

  // on_configure: QP_TSID (owns the robot model) and LIPTaskPlanner from the config yaml, control
  // period dt_; false (error logged) if a file / value is invalid
  virtual bool setupWbc ();

  // on_configure, after setupWbc (needs the model and dt_): StateEstimator and ContactFusion;
  // false (error logged) if a frame is invalid
  virtual bool setupStateEstimate ();

  // on_activate, after the bumpless hold and a first reading (sensor_ is valid): start the
  // estimators and the planner from the current state
  virtual void resetModules ();

  // update(): sensor_ -> estimated state q_, v_ and contact_probability_ (for the next cycle)
  virtual void estimate ();

  // update(): estimated state -> joint command q_cmd_ / v_cmd_ / tau_cmd_ (Pinocchio order);
  // false = no command this cycle, the last one is held
  virtual bool computeCommand (const rclcpp::Time &time);

  // on_configure: sizes the per-cycle buffers from the robot model (after setupWbc)
  void AllocateBuffers ();

  // robot model of all modules (owned by QP_TSID for now)
  tsid::robots::RobotWrapper &robotModel () { return this->qp_tsid_controller_->getRobotWrapper(); }

  // control period of all modules [s] (LIPPlanner sampling_frequency), set by setupWbc
  double dt_ = 0.0;

  // Parameters (declared in on_init, read in on_configure; defaults in ControllerParams, the WBC
  // yaml default in on_init, overridden by controllers.yaml)
  ControllerParams params_; //struct type


  // Modules, created in on_configure(): their constructors need files (yaml, URDF) that are only
  // known there, and pluginlib constructs the controller without arguments, so they cannot be
  // plain members. Order matters: qp_tsid_controller_ owns the robot model that the estimators
  // reference, so it is declared first (members are destroyed in reverse order).
  std::unique_ptr<QP_TSID> qp_tsid_controller_;      // tsid qp module (owns the robot wrapper)
  std::unique_ptr<LIPTaskPlanner> lip_task_planner_; // LIP task planner module
  std::unique_ptr<StateEstimator> state_estimator_;  // state estimator module
  std::unique_ptr<ContactFusion> contact_fusion_;    // contact fusion module

  // Joint order maps ros2_control (params_.joints) <-> Pinocchio (q[7:], v[6:], tau), built in
  // on_configure once the robot model exists
  JointMap joint_map_;

  // ros2_control interfaces (joints, IMU, motors enabled): claim, bind, read, write, hold. Created
  // in on_configure (needs the parameters and the joint map); works in Pinocchio order.
  std::unique_ptr<RobotInterface> robot_;

  // Per-cycle buffers, sized once in on_configure() (AllocateBuffers): update() runs in the
  // real-time loop and only writes into them, it never allocates memory.
  // Joint vectors are in Pinocchio order unless noted (see joint_map_).
  RobotSensor sensor_;                    // joint q / dq / tau + IMU of this cycle (estimator input)
  Eigen::VectorXd q_;                     // configuration [base pos 3, quat 4 (x y z w), joints] (nq)
  Eigen::VectorXd v_;                     // velocity [base twist 6 (base frame), joints] (nv)
  Eigen::VectorXd contact_probability_;   // [left, right] in [0, 1]: from the contact fusion of the
                                          // previous cycle, used by the state estimator
  Eigen::VectorXd dv_;                    // accelerations solved by TSID (nv)
  Eigen::VectorXd tau_;                   // joint torques solved by TSID (na)
  Eigen::VectorXd q_cmd_;                 // joint command to send (robot_->write): position (na)
  Eigen::VectorXd v_cmd_;                 // velocity (na)
  Eigen::VectorXd tau_cmd_;               // feedforward torque (na)
};

}  // namespace biped_controllers
