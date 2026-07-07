#include "legged_controllers/command/TargetTrajectoriesCalculator.h"

#include <cmath>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

namespace legged {

TargetTrajectoriesCalculator::TargetTrajectoriesCalculator(const std::string& referenceFile,
                                                           const ocs2::CentroidalModelInfo& info,
                                                           ocs2::PinocchioInterface pinocchioInterface,
                                                           scalar_t mpcHorizon)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(info),
      mass_(pinocchio::computeTotalMass(pinocchioInterface_.getModel())),
      mpcHorizon_(mpcHorizon) {
  const size_t nJoints = info_.actuatedDofNum;
  targetJointState_.resize(nJoints);

  ocs2::loadData::loadCppDataType(referenceFile, "defaultBaseHeight",          defaultBaseHeight_);
  ocs2::loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX",   maxDisplacementVelocityX_);
  ocs2::loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY",   maxDisplacementVelocityY_);
  ocs2::loadData::loadCppDataType(referenceFile, "maxRotationVelocity",        maxRotationVelocity_);

  targetJointState_ = ocs2::centroidal_model::loadDefaultJointState(nJoints, referenceFile);
}

/*******************************************************************************/

ocs2::TargetTrajectories TargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(
    const vector4_t& commandedVelocities,
    scalar_t initTime,
    const vector_t& initState) {
  // ── 1. Current base pose (zero out pitch/roll) ──────────────────────────
  vector6_t currentPoseTarget = getCurrentBasePoseTarget(initState);

  // ── 2. Scale, filter, rotate command to world frame ─────────────────────
  vector4_t commVelGlobal = scaleAndFilterVelCommand(commandedVelocities, currentPoseTarget(3));

  // Desired pelvis height comes directly from the command (not normalised)
  const scalar_t targetHeight = commVelGlobal(2);

  // ── 3. Target centroidal momentum = desired velocity ─────────────────────
  // Normalised centroidal state[0:3] ≈ CoM linear velocity;
  // state[5] ≈ normalised angular momentum z.
  vector6_t targetMomentum;
  targetMomentum << commVelGlobal(0), commVelGlobal(1), 0.0, 0.0, 0.0, commVelGlobal(3) / mass_;

  // ── 4. Current base linear/yaw velocity from centroidal momentum matrix ─
  const vector_t q = getGeneralizedCoordinates(initState);
  ocs2::updateCentroidalDynamics(pinocchioInterface_, info_, q);
  const auto& A  = ocs2::getCentroidalMomentumMatrix(pinocchioInterface_);
  const Eigen::Matrix<scalar_t, 6, 6> Ab     = A.leftCols<6>();
  const Eigen::Matrix<scalar_t, 6, 6> Ab_inv = ocs2::computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const vector6_t baseVel = Ab_inv * initState.head(6);

  // ── 5. Integrate base pose: current → intermediate (70% horizon) → final ─
  vector3_t avgVel;
  avgVel(0) = (baseVel(0) + commVelGlobal(0)) / 2.0;
  avgVel(1) = (baseVel(1) + commVelGlobal(1)) / 2.0;
  avgVel(2) = (baseVel(5) + commVelGlobal(3)) / 2.0;

  currentPoseTarget(2) = targetHeight;

  const scalar_t tIntermediate = 0.7 * mpcHorizon_;
  const vector6_t intermediatePose =
      integrateTargetBasePose(currentPoseTarget, avgVel, targetHeight, tIntermediate);

  avgVel(0) = commVelGlobal(0);
  avgVel(1) = commVelGlobal(1);
  avgVel(2) = commVelGlobal(3);
  const vector6_t finalPose =
      integrateTargetBasePose(intermediatePose, avgVel, targetHeight, mpcHorizon_ - tIntermediate);

  // ── 6. Pack into TargetTrajectories ─────────────────────────────────────
  const ocs2::scalar_array_t timeTrajectory{initTime,
                                            initTime + tIntermediate,
                                            initTime + mpcHorizon_};

  const size_t stateDim = initState.size();
  const size_t inputDim = info_.inputDim;

  ocs2::vector_array_t stateTrajectory(3, vector_t::Zero(stateDim));
  stateTrajectory[0] << targetMomentum, currentPoseTarget,  targetJointState_;
  stateTrajectory[1] << targetMomentum, intermediatePose,   targetJointState_;
  stateTrajectory[2] << targetMomentum, finalPose,          targetJointState_;

  const ocs2::vector_array_t inputTrajectory(3, vector_t::Zero(inputDim));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/*******************************************************************************/

TargetTrajectoriesCalculator::vector6_t
TargetTrajectoriesCalculator::getCurrentBasePoseTarget(const vector_t& state) const {
  // centroidal state: [h/m(6), base_pos(3), euler_ZYX(3), joints(12)]
  vector6_t pose = state.segment<6>(6);
  pose(4) = 0.0;  // zero pitch
  pose(5) = 0.0;  // zero roll
  return pose;
}

/*******************************************************************************/

TargetTrajectoriesCalculator::vector4_t
TargetTrajectoriesCalculator::scaleAndFilterVelCommand(const vector4_t& rawCmd, scalar_t currentYaw) {
  // Scale normalised [-1,1] velocities to physical units [m/s, rad/s].
  // desired_pelvis_height (index 2) is already in metres — pass through unchanged.
  vector4_t scaled;
  scaled(0) = rawCmd(0) * maxDisplacementVelocityX_;
  scaled(1) = rawCmd(1) * maxDisplacementVelocityY_;
  scaled(2) = rawCmd(2);  // height in metres, not normalised
  scaled(3) = rawCmd(3) * maxRotationVelocity_;

  // One-pole low-pass filter (matches ProceduralMpcMotionManager filter)
  commVelFiltered_ = commVelFiltered_ * kFilterAlpha + scaled * (1.0 - kFilterAlpha);

  // Rotate vx, vy from pelvis frame to world frame; height and yaw rate unchanged.
  vector4_t global = commVelFiltered_;
  global(0) = std::cos(currentYaw) * commVelFiltered_(0) - std::sin(currentYaw) * commVelFiltered_(1);
  global(1) = std::sin(currentYaw) * commVelFiltered_(0) + std::cos(currentYaw) * commVelFiltered_(1);

  return global;
}

/*******************************************************************************/

TargetTrajectoriesCalculator::vector6_t
TargetTrajectoriesCalculator::integrateTargetBasePose(const vector6_t& pose,
                                                       const vector3_t& averageVel,
                                                       scalar_t targetHeight,
                                                       scalar_t dt) const {
  vector6_t target = pose;
  target(0) += averageVel(0) * dt;
  target(1) += averageVel(1) * dt;
  target(2)  = targetHeight;   // height set directly (not integrated)
  target(3) += averageVel(2) * dt;
  target(4)  = 0.0;
  target(5)  = 0.0;
  return target;
}

/*******************************************************************************/

ocs2::vector_t TargetTrajectoriesCalculator::getGeneralizedCoordinates(const vector_t& state) const {
  // OCS2's updateCentroidalDynamics expects q in the same layout as the MPC
  // state position/orientation block — NOT pinocchio's standard quaternion format:
  //   q = [base_pos(3), base_euler_ZYX(3), joint_angles(nJoints)]  → size 18
  // (OCS2 converts Euler→quaternion internally before calling pinocchio.)
  const size_t nJoints = info_.actuatedDofNum;
  vector_t q(6 + nJoints);
  q.head<3>()     = state.segment<3>(6);   // base position [px, py, pz]
  q.segment<3>(3) = state.segment<3>(9);   // base ZYX euler [yaw, pitch, roll]
  q.tail(nJoints) = state.tail(nJoints);   // joint angles
  return q;
}

}  // namespace legged
