// Author: Van Thanh Nguyen - HUR group's Ph.D. student - GIST
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

namespace control_actions
{
    using namespace Eigen;

    // Rotation matrices
    Matrix3d rot_x(double theta);
    Matrix3d rot_y(double theta);
    Matrix3d rot_z(double theta);

    // Helper: clamp safely for asin/acos with tolerance
    double safeClamp(double x, double tol = 1e-9);
    // Helper: check if value is NaN or Inf
    inline bool isInvalid(double x);
    // Inverse kinematics from hip to foot
    std::vector<double> InvHip2Foot(
        const Matrix3d &R,
        const Vector3d &p,
        const std::vector<double> &current_joint_angle,
        bool isRight);
}
