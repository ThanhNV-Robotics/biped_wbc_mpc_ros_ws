#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace legged {

/**
 * Compute feedforward joint torques via inverse dynamics.
 *
 * Given the measured robot state and the MPC-optimal contact forces, solves
 * the floating-base equations of motion analytically (no QP required):
 *
 *   M * q̈ + h = J^T * F_contact + S^T * τ
 *
 * Steps:
 *   1. Compute base acceleration from the floating-base rows (closed form).
 *   2. Compute joint torques from the actuated-joint rows.
 *
 * @param rbdStateMeasured  Full rigid-body state from the estimator:
 *                          [euler_ZYX(3), base_pos(3), joints(n),
 *                           ang_vel(3),   lin_vel(3),  joint_vel(n)]
 * @param qdd_joints        Desired joint accelerations (12). Pass zeros for
 *                          standing/slow walking; add a PD correction for
 *                          aggressive tracking.
 * @param contactForces     MPC optimal contact forces packed as
 *                          [F_left(3), F_right(3), ...] (size 3×numContacts).
 * @param info              Centroidal model info (dimensions, frame indices).
 * @param pinocchioInterface  Dedicated pinocchio instance — must NOT be shared
 *                            with the MPC thread (crba/nle overwrite data).
 * @return Joint torques (size actuatedDofNum).
 */
ocs2::vector_t computeInverseDynamicsTorques(const ocs2::vector_t& rbdStateMeasured,
                                              const ocs2::vector_t& qdd_joints,
                                              const ocs2::vector_t& contactForces,
                                              const ocs2::CentroidalModelInfo& info,
                                              ocs2::PinocchioInterface& pinocchioInterface);

}  // namespace legged
