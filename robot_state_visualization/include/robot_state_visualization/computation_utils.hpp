// Author: Van Thanh Nguyen - HUR group's Ph.D. student - GIST
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

#ifndef CONTROL_ACTIONS_COMPUTATION_UTILS_HPP_
#define CONTROL_ACTIONS_COMPUTATION_UTILS_HPP_

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
    Eigen::VectorXd InvHip2Foot(
        const Matrix3d &R,
        const Vector3d &p,
        const VectorXd &current_joint_angle,
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

    class KalmanFilter
    {

        public:
            KalmanFilter(
                const Eigen::MatrixXd &F, // System dynamics.
                const Eigen::MatrixXd &B, // Control.
                const Eigen::MatrixXd &H, // Output.
                const Eigen::MatrixXd &Q, // Estimate error covariance.
                const Eigen::MatrixXd &R, // Measurement noise covariance.
                const Eigen::MatrixXd &P,  // Process noise covariance.
                const Eigen::VectorXd &x0  // Process noise covariance.
            );
            ~KalmanFilter();


            VectorXd predict();//u = 0
            void update(const Eigen::VectorXd &y);
            Eigen::VectorXd kalman_filt (Eigen::VectorXd input_measurement);            

        private:
            Eigen::MatrixXd F, B, H, Q, R, P;
            Eigen::MatrixXd I;  // unit matrix.

            Eigen::VectorXd x; // estimated state.
    }; // class Kalman Filter
} // namespace control_actions
#endif  // CONTROL_ACTIONS_COMPUTATION_UTILS_HPP_