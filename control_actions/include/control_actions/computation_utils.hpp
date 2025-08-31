// Author: Van Thanh Nguyen - HUR group's Ph.D. student - GIST
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions> 

namespace control_actions
{
    using namespace Eigen;

    // Rotation matrices
    Matrix3d rot_x(double theta);
    Matrix3d rot_y(double theta);
    Matrix3d rot_z(double theta);

    // Helper: clamp safely for asin/acos with tolerance
    double safeClamp(double x);
    // Helper: check if value is NaN or Inf
    inline bool isInvalid(double x);
    // Inverse kinematics from hip to foot
    std::vector<double> InvHip2Foot(
        const Matrix3d &R,
        const Vector3d &p,
        const std::vector<double> &current_joint_angle,
        bool isRight);
    void c2d(const Eigen::MatrixXd &A_c,
             const Eigen::MatrixXd &B_c,
             const Eigen::MatrixXd &C_c,
             const Eigen::MatrixXd &D_c,
             double dt,
             Eigen::MatrixXd &A_d,
             Eigen::MatrixXd &B_d,
             Eigen::MatrixXd &C_d,
             Eigen::MatrixXd &D_d);
}
