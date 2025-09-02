#include <control_actions/standing_balance_control.hpp>
namespace control_actions
{
    StandingBalanceController::StandingBalanceController(uint16_t control_frequency)
        : kalman_filter_(A_kalman_, B_kalman_, C_kalman_, P_kalman_, R_kalman_, Q_kalman_) // initialize here
    {
        control_frequency_ = control_frequency;
        dt_ = 1.0/control_frequency_;

        // compute discete system matrix
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
        Adis = Mexp.block(0, 0, n, n);
        Bdis = Mexp.block(0, n, n, m);
    }
}