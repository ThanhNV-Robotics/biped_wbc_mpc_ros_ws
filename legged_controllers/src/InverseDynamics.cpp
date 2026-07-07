#include <pinocchio/fwd.hpp>

#include "legged_controllers/InverseDynamics.h"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged {

using scalar_t = ocs2::scalar_t;
using vector_t = ocs2::vector_t;
using matrix_t = ocs2::matrix_t;

ocs2::vector_t computeInverseDynamicsTorques(const vector_t& rbdStateMeasured,
                                              const vector_t& qdd_joints,
                                              const vector_t& contactForces,
                                              const ocs2::CentroidalModelInfo& info,
                                              ocs2::PinocchioInterface& pinocchioInterface)
{
  const size_t nq       = info.generalizedCoordinatesNum;  // 18
  const size_t nActuated = info.actuatedDofNum;             // 12

  // ── Build pinocchio q, qd from rbdStateMeasured ──────────────────────────
  // rbdState layout: [euler_ZYX(3), base_pos(3), joints(n),
  //                   ang_vel_world(3), lin_vel_world(3), joint_vel(n)]
  vector_t q(nq), qd(nq);
  q.head<3>()       = rbdStateMeasured.segment<3>(3);         // base position
  q.segment<3>(3)   = rbdStateMeasured.head<3>();             // euler ZYX [yaw,pitch,roll]
  q.tail(nActuated) = rbdStateMeasured.segment(6, nActuated); // joint angles

  qd.head<3>()       = rbdStateMeasured.segment<3>(nq + 3);   // linear velocity
  qd.segment<3>(3)   = ocs2::getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      q.segment<3>(3), rbdStateMeasured.segment<3>(nq));       // euler dot
  qd.tail(nActuated) = rbdStateMeasured.segment(nq + 6, nActuated); // joint vel

  const auto& model = pinocchioInterface.getModel();
  auto& data        = pinocchioInterface.getData();

  // ── Inertia matrix M and nonlinear effects h ─────────────────────────────
  pinocchio::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, q, qd);

  // ── Contact Jacobians → external forces in joint space ───────────────────
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  vector_t F_ext = vector_t::Zero(nq);
  for (size_t i = 0; i < info.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> J(6, nq);
    J.setZero();
    pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[i],
                                pinocchio::LOCAL_WORLD_ALIGNED, J);
    // 3-DOF contact: only translational rows
    F_ext += J.topRows<3>().transpose() * contactForces.segment<3>(3 * i);
  }

  // ── Base acceleration (floating-base rows, closed form) ──────────────────
  // M_bb * q̈_base = F_ext_base - h_base - M_bj * q̈_j
  const Eigen::Matrix<scalar_t, 6, 6> M_bb = data.M.topLeftCorner<6, 6>();
  const matrix_t                       M_bj = data.M.block(0, 6, 6, nActuated);
  const vector_t rhs_base = F_ext.head<6>() - data.nle.head<6>() - M_bj * qdd_joints;
  const vector_t qdd_base = M_bb.ldlt().solve(rhs_base);

  // ── Joint torques (actuated rows) ─────────────────────────────────────────
  // τ = M_jb * q̈_base + M_jj * q̈_j + h_j - F_ext_j
  const matrix_t M_jb = data.M.block(6, 0, nActuated, 6);
  const matrix_t M_jj = data.M.block(6, 6, nActuated, nActuated);
  return M_jb * qdd_base + M_jj * qdd_joints + data.nle.tail(nActuated) - F_ext.tail(nActuated);
}

}  // namespace legged
