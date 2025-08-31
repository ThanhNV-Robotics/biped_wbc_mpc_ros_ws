#include "computation_utils.hpp"
#include <Eigen/Dense>
namespace control_actions
{
    class StandingBalanceController
    {
    public:
        StandingBalanceController(uint16_t control_frequency);

    private:
        const double Tzmp = 0.01; // sec, time const of the ZMP feedback
        const double zc = 0.62;   // m, CoM height ~ pelvis height
        const double g = 9.81;    // m/s2, gravity const

        uint16_t control_frequency_;
        const double dt = 1.0 / control_frequency_; // sec, control loop period
        // Continuous time state-space matrices
        Eigen::Matrix<double, 6, 6> Act = (Eigen::Matrix<double, 6, 6>() << -1.0 / Tzmp, 0, 1.0 / Tzmp, 0, 0, 0,
                                           0, -1.0 / Tzmp, 0, 0, 1.0 / Tzmp, 0,
                                           0, 0, 0, 1, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 1,
                                           0, 0, 0, 0, 0, 0)
                                              .finished();
        Eigen::Matrix<double, 6, 2> Bct = (Eigen::Matrix<double, 6, 2>() << -zc / (g * Tzmp), 0,
                                           0, -zc / (g * Tzmp),
                                           0, 0,
                                           1, 0,
                                           0, 0,
                                           0, 1)
                                              .finished();
        Eigen::Matrix<double, 2, 6> Cct = (Eigen::Matrix<double, 2, 6>() << 1, 0, 0, 0, 0, 0,
                                           0, 1, 0, 0, 0, 0)
                                              .finished();

        Eigen::Matrix<double, 2, 2> Dct = Eigen::Matrix<double, 2, 2>::Zero();
    };
} // namespace control_actions