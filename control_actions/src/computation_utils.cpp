#include <control_actions/computation_utils.hpp>
namespace control_actions
{
    // Rotation matrices
    Matrix3d rot_x(double theta)
    {
        Matrix3d R;
        R << 1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);
        return R;
    }

    Matrix3d rot_y(double theta)
    {
        Matrix3d R;
        R << cos(theta), 0, sin(theta),
            0, 1, 0,
            -sin(theta), 0, cos(theta);
        return R;
    }

    Matrix3d rot_z(double theta)
    {
        Matrix3d R;
        R << cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0, 0, 1;
        return R;
    }

    // Helper: clamp safely for asin/acos with tolerance
    double safeClamp(double x)

    {
        double tol = 1e-9;
        if (x > 1.0 + tol || x < -1.0 - tol)
        {
            // way outside domain → invalid
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (x > 1.0)
            return 1.0;
        if (x < -1.0)
            return -1.0;
        return x;
    }

    // Helper: check if value is NaN or Inf
    inline bool isInvalid(double x)
    {
        return std::isnan(x) || std::isinf(x);
    }

    // Inverse kinematics from hip to foot
    std::vector<double> InvHip2Foot(
        const Matrix3d &R,
        const Vector3d &p,
        const std::vector<double> &current_joint_angle,
        bool isRight)
    {
        // isRight = true -> compute for right leg
        // isRight = false -> compute for left leg
        // Link lengths
        const double L1 = 0.324; // thigh
        const double L2 = 0.335; // shank
        const double L3 = 0.0;   // ankle offset

        double delta = isRight ? M_PI / 12.0 : -M_PI / 12.0; // 15 deg in rad

        Matrix3d hip_transform = rot_x(delta).transpose();

        // Inverse rotation and transformed position
        Matrix3d R_ref = R.transpose();
        Vector3d p_ref = -R_ref * p;

        Matrix3d R_adj = hip_transform * R;

        double px = p_ref(0);
        double py = p_ref(1);
        double pz = p_ref(2);

        // Compute knee angle (theta4)
        double cos4 = (L1 * L1 + L2 * L2 - ((px + L3) * (px + L3) + py * py + pz * pz)) / (2 * L1 * L2);
        cos4 = safeClamp(cos4);
        if (isInvalid(cos4))
        {
            std::cerr << "Warning: Unreachable target (cos4 out of range)" << std::endl;
            return current_joint_angle;
        }
        double th4 = M_PI - acos(cos4);

        // Ankle roll (theta5)
        double th5 = atan2(py, pz + L3);

        // Ankle pitch (theta6)
        double C = sqrt(px * px + py * py + pz * pz);
        double alpha_arg = (L1 * sin(M_PI - th4)) / C;
        alpha_arg = safeClamp(alpha_arg);
        if (isInvalid(alpha_arg))
        {
            std::cerr << "Warning: Unreachable target (alpha out of range)" << std::endl;
            return current_joint_angle;
        }
        double alpha = asin(alpha_arg);
        double th6 = -atan2(px, ((pz >= 0) ? 1 : -1) * sqrt(py * py + pz * pz)) - alpha;

        // Compute hip angles
        Matrix3d RHS = R_adj * rot_y(-th6) * rot_x(-th5) * rot_y(-th4);

        double th1 = atan2(RHS(0, 2), RHS(2, 2));
        double th2 = asin(safeClamp(-RHS(1, 2))) + delta;
        double th3 = atan2(RHS(1, 0), RHS(1, 1));

        // Collect all angles
        std::vector<double> result = {th1, th2, th3, th4, th5, th6};

        // Check if any angle is invalid
        for (double ang : result)
        {
            if (isInvalid(ang))
            {
                std::cerr << "Warning: Computed joint angle invalid (NaN/Inf)" << std::endl;
                return current_joint_angle;
            }
        }

        return result;
    }

    // Discretize continuous state-space (A, B) with timestep dt
    void c2d(const Eigen::MatrixXd &A_c,
             const Eigen::MatrixXd &B_c,
             const Eigen::MatrixXd &C_c,
             const Eigen::MatrixXd &D_c,
             double dt,
             Eigen::MatrixXd &A_d,
             Eigen::MatrixXd &B_d,
             Eigen::MatrixXd &C_d,
             Eigen::MatrixXd &D_d)
    {
        int n = A_c.rows();
        int m = B_c.cols();

        // Build augmented matrix
        Eigen::MatrixXd M(n + m, n + m);
        M.setZero();
        M.block(0, 0, n, n) = A_c;
        M.block(0, n, n, m) = B_c;

        // Compute matrix exponential
        Eigen::MatrixXd Mexp = (M * dt).exp();

        // Extract Ad and Bd
        A_d = Mexp.block(0, 0, n, n);
        B_d = Mexp.block(0, n, n, m);

        // C and D remain unchanged
        C_d = C_c;
        D_d = D_c;
    }
}


// // === Test main inverse kinematics computation===
// int main() {
//     // Foot rotation = Identity (parallel to ground)
//     Matrix3d R_foot = Matrix3d::Identity();

//     // Desired foot position
//     double pelvis_height = 0.62;  // example value
//     Vector3d p_foot(0.0, 0.0, -pelvis_height);

//     // Current joint angles (assume zero posture)
//     std::vector<double> current_joint_angle = {0,0,0,0,0,0};

//     // Compute IK
//     std::vector<double> result = InvHip2Foot(R_foot, p_foot, current_joint_angle, true);

//     // Print result
//     std::cout << "Joint Angles (rad): ";
//     for (double ang : result) {
//         std::cout << ang << " ";
//     }
//     std::cout << std::endl;

//     return 0;
// }

// Test c2d function
using namespace control_actions;
int main ()
{
        
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

        Eigen::MatrixXd A_d;
        Eigen::MatrixXd B_d;
        Eigen::MatrixXd C_d;
        Eigen::MatrixXd D_d;
        control_actions::c2d(Act, Bct, Cct, Dct, 0.004, A_d, B_d, C_d, D_d);
        std::cout << "A_ct: \n" << Act << std::endl;
        std::cout << "B_ct: \n" << Bct << std::endl;
        std::cout << "C_ct: \n" << Cct << std::endl;
        std::cout << "D_ct: \n" << Dct << std::endl;

        std::cout << "A_d: \n" << A_d << std::endl;
        std::cout << "B_d: \n" << B_d << std::endl;
        std::cout << "C_d: \n" << C_d << std::endl;
        std::cout << "D_d: \n" << D_d << std::endl;
}