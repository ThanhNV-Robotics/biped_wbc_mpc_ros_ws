//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_estimation/LinearKalmanFilter.h>

#include <legged_interface/LeggedInterface.h> // this package handle the robot Optimal Control interface: constraints, cost, reference trajectory, etc. for the MPC and MRT

namespace legged {

class LeggedController : public controller_interface::ControllerInterface
{
public:
  LeggedController() = default;
  ~LeggedController() override = default;

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
  // for State Estimation
  virtual void updateStatesEstimatate (const rclcpp::Time& time, const rclcpp::Duration& period);
  // set up function for the State Estimation
  virtual void setupStateEstimate (const std::string& taskFile, bool verbose);

  // // Set up robot interface
  // // taskfile: path to the task file defining the MPC (task.info)
  // //urdffile: path to the robot URDF file 
  // //referenceFile: path to the reference trajectory file (reference.info)
  // virtual void setupLeggedInterface (const std::string& taskfile, const std::string& urdffile, const std::string& referenceFile, bool verbose);

  // // Set up function for the MRT
  // // MRT: Model Reference Tracking: transform the MPC output into the actual control command (e.g., PD gains) and transforms observations to MPC states.
  // virtual void setupMrt ();

  // // Set up function for the MPC
  // virtual void setupMpc ();

  

  // Interface attributes
  std::shared_ptr<LeggedInterface> legged_interface_;

  // State Estimation
  
  std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  SystemObservation currentObservation_; // mpc observation struct
  // // SystemObservation is a struc defined in ocs2_mpc which contains the time, state, input, and
  //  mode information for the MPC. The state estimation will update the currentObservation_ and
  //   then the MPC will use it as the current state to query for the optimal control input. The
  //    measuredRbdState_ is the state measured from the robot, which may be different from the
  //     state used in the MPC (e.g., the MPC may use a reduced-order model). The rbdConversions_ is
  //      used to convert between the measured RBD state and the state used in the MPC.
    // struct SystemObservation {
    //   size_t mode = 0; ,<=> contact mode information for the MPC
    //   scalar_t time = 0.0;
    //   vector_t state;
    //   vector_t input;

    //   friend void swap(SystemObservation& a, SystemObservation& b) noexcept;
    // };

};





class LeggedCheaterController : public LeggedController
{
public:
  LeggedCheaterController() = default;
  ~LeggedCheaterController() override = default;

  controller_interface::CallbackReturn on_init() override;
};

}  // namespace legged
