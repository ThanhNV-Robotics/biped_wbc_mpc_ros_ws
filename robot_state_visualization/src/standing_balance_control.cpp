#include "robot_state_visualization/standing_balance_control.hpp"
namespace control_actions
{
    StandingBalanceController::StandingBalanceController(uint16_t control_frequency)
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
        Adis_ = Mexp.block(0, 0, n, n);
        Bdis_ = Mexp.block(0, n, n, m);
    }
    Eigen::VectorXd StandingBalanceController::compute_balancing_motion (double zmp_x, double zmp_y, VectorXd joint_fb_angles)
    {
        // extract left_leg_current_joint_angles and right angles from the joint feedback angles
        Eigen::VectorXd left_leg_current_joint_angles = joint_fb_angles.head(6);
        Eigen::VectorXd right_leg_current_joint_angles = joint_fb_angles.tail(6);

        // stack zmp_x and zmp_y to (2,1) vector
        Eigen::VectorXd zmp(2);
        zmp << zmp_x, zmp_y;

        // stack x_state = np.vstack((zmp_fb, self.xc_d))
        Eigen::VectorXd x_state(4);
        x_state << zmp, xc_d;

        //compute control input u = -self.K@x_state
        Eigen::VectorXd u = -K_ * x_state;
        
        //calculate next time step state x_update = self.Adis@x_state  + self.Bdis@u
        Eigen::VectorXd x_update = Adis_ * x_state + Bdis_ * u;

        //update current state self.xc_d = x_update[2:]
        xc_d = x_update.tail(2);

        //extract com's desired position com_x = x_update[2].item()
        double com_x = x_update[2];
        double com_y = x_update[3];

        // compute inverse kinematics
        // init foot position and orientation
        Vector3d p_foot(0.0, 0.0, -zc); // foot position w.r.t the hip, must pass zc as the hip_height constrain
        Matrix3d R_foot = Matrix3d::Identity(); // foot orientation, parallel to the ground

        // Compute Inverse Kinematics
        Eigen::VectorXd left_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, left_leg_current_joint_angles, false);  // left leg
        Eigen::VectorXd right_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, right_leg_current_joint_angles, true); // right leg

        // Stack left and right leg joint angles into one vector, left leg first
        Eigen::VectorXd target_joint_angles;
         target_joint_angles.resize(left_leg_target_joint_angles.size() +
                           right_leg_target_joint_angles.size()); // must resize to a proper length first

        target_joint_angles << left_leg_target_joint_angles, right_leg_target_joint_angles;
        return target_joint_angles;
    }
    Eigen::Vector3d StandingBalanceController::compute_zmp (Vector3d com_pos, Vector3d acceleration)
    {
        double x = com_pos[0];
        double y = com_pos[1];
        double z = com_pos[2];

        VectorXd filted_acceleration = kalman_filt(acceleration);
        // VectorXd filted_acceleration = acceleration;
        double acc_x = filted_acceleration[0];
        double acc_y = filted_acceleration[1];
        double acc_z = filted_acceleration[2];

        double px = x-(z*acc_x)/g; // unit in m/s2
        double py = y-(z*acc_y)/g; // unit in m/s2

        // double px = x-(z*acc_x); // unit in g
        // double py = y-(z*acc_y); // unit in g

        double pz = 0.0;

        Vector3d zmp;
        zmp << px, py, pz;
        return zmp;
    }
    Eigen::VectorXd StandingBalanceController::kalman_filt (VectorXd measurement)
    {
        return kalman_filter_.kalman_filt(measurement);
    }
}