#include "computation_utils.hpp"
#include <Eigen/Dense>
namespace control_actions
{
    class StandingBalanceController
    {
    public:
        StandingBalanceController(uint16_t control_frequency);
        void set_control_frequency (uint16_t control_frequency)
        {
            control_frequency_ = control_frequency;
            dt_ = 1.0 / static_cast<double>(control_frequency_);
            // re-compute system discrete matrix
            int n = Act.rows();
            int m = Bct.cols();

            // Build augmented matrix
            Eigen::MatrixXd M(n + m, n + m);
            M.setZero();
            M.block(0, 0, n, n) = Act;
            M.block(0, n, n, m) = Bct;

            // Compute matrix exponential
            Eigen::MatrixXd Mexp = (M * dt_).exp();

            // Extract Ad and Bd
            Adis_ = Mexp.block(0, 0, n, n);
            Bdis_ = Mexp.block(0, n, n, m);
        }
        Eigen::VectorXd compute_balancing_motion (double zmp_x, double zmp_y, VectorXd joint_fb_angles);
        Eigen::Vector3d  compute_zmp (Vector3d com_pos, Vector3d acceleration);
        Eigen::VectorXd kalman_filt (VectorXd measurement);
        double get_zc() const { return zc; }

    private:
        const double Tzmp = 0.01; // sec, time const of the ZMP feedback
        const double zc = 0.62;   // m, CoM height ~ pelvis height
        const double g = 9.81;    // m/s2, gravity const
        Eigen::VectorXd xc_d = Eigen::VectorXd::Zero(4,1); // desired com position and velocity in x and y direction

        uint16_t control_frequency_; // Hz, default value
        double dt_; // sec, control loop period
        // Continuous time state-space matrices
        const Eigen::Matrix<double, 6, 6> Act = (Eigen::Matrix<double, 6, 6>() << -1.0 / Tzmp, 0, 1.0 / Tzmp, 0, 0, 0,
                                           0, -1.0 / Tzmp, 0, 0, 1.0 / Tzmp, 0,
                                           0, 0, 0, 1, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 1,
                                           0, 0, 0, 0, 0, 0)
                                              .finished();
        const Eigen::Matrix<double, 6, 2> Bct = (Eigen::Matrix<double, 6, 2>() << -zc / (g * Tzmp), 0,
                                           0, -zc / (g * Tzmp),
                                           0, 0,
                                           1, 0,
                                           0, 0,
                                           0, 1)
                                              .finished();
        // Discrete State Space System
        Eigen::MatrixXd Adis_;
        Eigen::MatrixXd Bdis_;
        // State feedback gain matrix, calculated from python file
        // it depends on Adis, Bdis, Q and R matrix
        // here we dont add Q and R as attributes
        Eigen::Matrix<double, 2, 6> K_ = (Eigen::Matrix<double, 2, 6>() <<
            -3.36492283e+01, -2.60109806e-14, 9.22916451e+01, 3.08056903e+01, -1.46315623e-14, -1.69549718e-13,
            -2.25608521e-14, -3.36600309e+01, -1.51937747e-12, -1.50089438e-13, 9.23161487e+01, 3.08127113e+01
        ).finished();

        // Q and R matrices of kalman filter
        // Kalman filter matrices
        const Eigen::MatrixXd F_kalman_ = Eigen::MatrixXd::Identity(3, 3);  // State transition matrix (identity for acceleration filter)
        const Eigen::MatrixXd B_kalman_ = Eigen::MatrixXd::Zero(3, 1);  // Measurement matrix (identity)
        const Eigen::MatrixXd H_kalman_ = Eigen::MatrixXd::Identity(3, 3);
        const Eigen::MatrixXd Q_kalman_ =  0.05*Eigen::MatrixXd::Identity(3, 3);

        // tuning matrix of Kalman Filter
        const Eigen::MatrixXd R_kalman_ = 2.5 * Eigen::MatrixXd::Identity(3, 3);   // Measurement noise covariance
        const Eigen::MatrixXd P_kalman_ = Eigen::MatrixXd::Identity(3, 3);  // Process noise covariance
        const Eigen::VectorXd x0_kalman_ = Eigen::VectorXd::Zero(3,1); // initial state

        KalmanFilter kalman_filter_ = KalmanFilter(F_kalman_, H_kalman_, B_kalman_, Q_kalman_,  R_kalman_, P_kalman_, x0_kalman_); // init an Kalman filter if needed
    };
} // namespace control_actions