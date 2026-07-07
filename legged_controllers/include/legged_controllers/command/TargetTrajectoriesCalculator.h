#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace legged {

/**
 * Converts a walking velocity command into an OCS2 TargetTrajectories suitable
 * for the centroidal MPC cost function.
 *
 * Input command vector4_t = [vx_norm, vy_norm, desired_pelvis_height_m, omega_z_norm]
 *   vx_norm / vy_norm / omega_z_norm : normalised [-1, 1]; scaled internally by
 *   maxDisplacementVelocityX/Y and maxRotationVelocity loaded from reference.info.
 *   desired_pelvis_height_m          : directly in metres (e.g. 0.65–0.75).
 *
 * Ported from humanoid_common_mpc TargetTrajectoriesCalculatorBase +
 * humanoid_centroidal_mpc CentroidalMpcTargetTrajectoriesCalculator.
 */
class TargetTrajectoriesCalculator {
 public:
  using scalar_t  = ocs2::scalar_t;
  using vector_t  = ocs2::vector_t;
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;
  using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;

  TargetTrajectoriesCalculator(const std::string& referenceFile,
                                const ocs2::CentroidalModelInfo& info,
                                ocs2::PinocchioInterface pinocchioInterface,
                                scalar_t mpcHorizon);

  ocs2::TargetTrajectories commandedVelocityToTargetTrajectories(
      const vector4_t& commandedVelocities,
      scalar_t initTime,
      const vector_t& initState);

 private:
  // Extract base pose [px, py, pz, yaw, 0, 0] from centroidal state.
  vector6_t getCurrentBasePoseTarget(const vector_t& state) const;

  // Scale normalised [-1,1] command by max velocities, then apply a one-pole
  // low-pass filter, then rotate from pelvis frame to world frame.
  vector4_t scaleAndFilterVelCommand(const vector4_t& rawCmd, scalar_t currentYaw);

  // Kinematic integration of base pose over dt.
  vector6_t integrateTargetBasePose(const vector6_t& pose,
                                    const vector3_t& averageVel,
                                    scalar_t targetHeight,
                                    scalar_t dt) const;

  // Build pinocchio generalised coordinates q from centroidal state.
  vector_t getGeneralizedCoordinates(const vector_t& state) const;

  ocs2::PinocchioInterface pinocchioInterface_;
  const ocs2::CentroidalModelInfo& info_;
  scalar_t mass_;
  scalar_t mpcHorizon_;

  // Loaded from reference.info
  scalar_t defaultBaseHeight_{0.75};
  vector_t targetJointState_;
  scalar_t maxDisplacementVelocityX_{0.6};
  scalar_t maxDisplacementVelocityY_{0.3};
  scalar_t maxRotationVelocity_{0.6};

  // Low-pass filter state (updated in-place each call)
  vector4_t commVelFiltered_{vector4_t::Zero()};
  static constexpr scalar_t kFilterAlpha = 0.8;  // smoothing coefficient [0,1)
};

}  // namespace legged
