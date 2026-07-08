// Standalone test for computeInverseDynamicsTorques().
//
// Loads the robot model, sets a known standing state, applies gravity-split
// contact forces, and prints the resulting joint torques. Run this to verify
// the Pinocchio computation is correct before enabling the full controller.
//
// Usage:
//   ros2 run legged_controllers test_pinocchio_id \
//     --ros-args -p urdf_file:=/path/to/biped_robot_12dof.urdf \
//                -p task_file:=/path/to/task.info \
//                -p reference_file:=/path/to/reference.info

#include <pinocchio/fwd.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>

#include <legged_interface/LeggedInterface.h>
#include "legged_controllers/InverseDynamics.h"

using namespace legged;
using namespace ocs2;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_pinocchio_id");

  const auto urdfFile      = node->declare_parameter<std::string>("urdf_file",      "");
  const auto taskFile      = node->declare_parameter<std::string>("task_file",       "");
  const auto referenceFile = node->declare_parameter<std::string>("reference_file",  "");

  if (urdfFile.empty() || taskFile.empty() || referenceFile.empty()) {
    RCLCPP_ERROR(node->get_logger(), "urdf_file, task_file, reference_file are required");
    return 1;
  }

  // ── Build OCS2 model ──────────────────────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "Building LeggedInterface...");
  LeggedInterface leggedInterface(taskFile, urdfFile, referenceFile);
  leggedInterface.setupOptimalControlProblem(taskFile, urdfFile, referenceFile, /*verbose=*/false);

  const auto& info     = leggedInterface.getCentroidalModelInfo();
  const size_t nq      = info.generalizedCoordinatesNum;   // 18
  const size_t nJoints = info.actuatedDofNum;              // 12

  RCLCPP_INFO(node->get_logger(), "Model: nq=%zu, nJoints=%zu, mass=%.2f kg, 3-DoF contacts=%zu",
              nq, nJoints, info.robotMass, info.numThreeDofContacts);

  // Dedicated pinocchio interface for ID (must not share with MPC thread)
  PinocchioInterface pinInterface = leggedInterface.getPinocchioInterface();

  // ── Load default joint angles from reference.info ─────────────────────────
  const vector_t defaultJoints = centroidal_model::loadDefaultJointState(nJoints, referenceFile);
  RCLCPP_INFO(node->get_logger(), "Default joints loaded: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, ...]",
              defaultJoints(0), defaultJoints(1), defaultJoints(2),
              defaultJoints(3), defaultJoints(4), defaultJoints(5));

  // ── Build a standing rbdState ─────────────────────────────────────────────
  // Layout: [euler_ZYX(3), base_pos(3), joints(nJ), ang_vel(3), lin_vel(3), joint_vel(nJ)]
  vector_t rbdState = vector_t::Zero(2 * nq);
  rbdState(3) = 0.0;   // base x
  rbdState(4) = 0.0;   // base y
  rbdState(5) = 0.765; // base z height
  rbdState.segment(6, nJoints) = defaultJoints;
  // velocities stay zero — static standing

  RCLCPP_INFO(node->get_logger(), "Test state: base_z=%.3f m, all velocities zero", rbdState(5));

  // ── Contact forces: gravity split equally across feet ─────────────────────
  const scalar_t totalWeight = info.robotMass * 9.81;
  vector_t contactForces = vector_t::Zero(3 * info.numThreeDofContacts);
  for (size_t i = 0; i < info.numThreeDofContacts; ++i) {
    contactForces(3 * i + 2) = totalWeight / info.numThreeDofContacts;  // fz only
  }
  RCLCPP_INFO(node->get_logger(), "Contact forces: fz = %.1f N per foot (total weight = %.1f N)",
              totalWeight / info.numThreeDofContacts, totalWeight);

  // ── Zero desired joint accelerations (quasi-static) ───────────────────────
  const vector_t qdd_joints = vector_t::Zero(nJoints);

  // ── Run inverse dynamics ──────────────────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "Running computeInverseDynamicsTorques...");
  const vector_t torques = computeInverseDynamicsTorques(rbdState, qdd_joints, contactForces, info, pinInterface);

  // ── Print results ──────────────────────────────────────────────────────────
  const std::vector<std::string> jointNames = {
    "left_hip_pitch",  "left_hip_roll",  "left_hip_yaw",
    "left_knee_pitch", "left_ankle_roll", "left_ankle_pitch",
    "right_hip_pitch", "right_hip_roll",  "right_hip_yaw",
    "right_knee_pitch","right_ankle_roll","right_ankle_pitch"
  };

  RCLCPP_INFO(node->get_logger(), "=== ID torques (quasi-static standing) ===");
  for (size_t j = 0; j < nJoints; ++j) {
    RCLCPP_INFO(node->get_logger(), "  %-22s  q=%.3f rad   τ=%.3f Nm",
                jointNames[j].c_str(), defaultJoints(j), torques(j));
  }

  // ── Sanity checks ─────────────────────────────────────────────────────────
  bool allFinite = torques.allFinite();
  bool reasonable = (torques.cwiseAbs().maxCoeff() < 500.0);  // < 500 Nm per joint
  RCLCPP_INFO(node->get_logger(), "=== Sanity checks ===");
  RCLCPP_INFO(node->get_logger(), "  All finite:    %s", allFinite  ? "PASS" : "FAIL — NaN/Inf detected");
  RCLCPP_INFO(node->get_logger(), "  Max |torque|:  %.1f Nm  (%s)",
              torques.cwiseAbs().maxCoeff(), reasonable ? "PASS < 500 Nm" : "FAIL — suspiciously large");
  RCLCPP_INFO(node->get_logger(), "  Left/right symmetry (hip pitch diff): %.4f Nm",
              std::abs(torques(0) - torques(6)));

  rclcpp::shutdown();
  return (allFinite && reasonable) ? 0 : 1;
}
