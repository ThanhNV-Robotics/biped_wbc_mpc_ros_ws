import numpy as np
from low_level_fcn import ankle_kinematics
import mujoco
import numpy as np
from mujoco.viewer import launch_passive

class PD_Control: # MIMO, joint position PD control
    def __init__(self, Kp, Kd):
        n = np.shape(Kp)[1]
        self.error_pre = np.zeros((n,1))
        self.Kp = Kp
        self.Kd = Kd
        
    def PD_Impedance_Calculate (self, ref_pos, ref_vel, fb_pos, fb_vel, tqr_ff): # MIMO
        
        # tqr_ff is the feedforward torque
        torque = self.Kp@(ref_pos - fb_pos) + self.Kd@(ref_vel - fb_vel) + tqr_ff
        return torque # return torque vector applied to robot's joints
    
    def PD_Calculate (self, ref_pos, fb_pos, tqr_ff, dt = 0.001): # MIMO
        
        # tqr_ff is the feedforward torque
        error = ref_pos - fb_pos
        error_d = (error - self.error_pre)/dt # finite derivative
        torque = self.Kp@error + self.Kd@error_d +tqr_ff
        self.error_pre = error

        return torque # return torque vector applied to robot's joints

def Get_Motor_Pos (mj_model, mj_data):
    n_motor = mj_model.nu
    motor_5_joint_id = mj_model.joint("Motor5_Joint").id
    motor_6_joint_id = mj_model.joint("Motor6_Joint").id

    theta_m5 = mj_data.qpos[motor_5_joint_id]
    theta_m6 = mj_data.qpos[motor_6_joint_id]

    return np.array([theta_m5,theta_m6]).reshape(n_motor,1)


def Get_Motor_Vel (mj_model, mj_data):
    motor_5_joint_id = mj_model.joint("Motor5_Joint").id
    motor_6_joint_id = mj_model.joint("Motor6_Joint").id
    theta_m5_vel = mj_data.qvel[motor_5_joint_id]
    theta_m6_vel = mj_data.qvel[motor_6_joint_id]

    return np.array([theta_m5_vel,theta_m6_vel])

def Get_Ankle_Vel (mj_model, mj_data):
    ankle_roll_joint_id = mj_model.joint("Right_Ankle_Roll").id # get joint id
    ankle_pitch_joint_id = mj_model.joint("Right_Ankle_Pitch").id 
    theta_5_d = mj_data.qvel[ankle_roll_joint_id]
    theta_6_d = mj_data.qvel[ankle_pitch_joint_id]

    return np.array([theta_5_d,theta_6_d])

def Get_Ankle_Pos (mj_model, mj_data):
    ankle_roll_joint_id = mj_model.joint("Right_Ankle_Roll").id # get joint id
    ankle_pitch_joint_id = mj_model.joint("Right_Ankle_Pitch").id 
    theta_5 = mj_data.qpos[ankle_roll_joint_id]
    theta_6 = mj_data.qpos[ankle_pitch_joint_id]

    return np.array([theta_5,theta_6])

def step_ref (t, T, ref_angl): # to generate referenc angle
    if  t <= T:
        angle = (6*(t/T)**5 - 15*(t/T)**4 + 10*(t/T)**3)*ref_angl
        velocity = (30*t**4/T**5 - 60*t**3/T**4 + 30*t**2/T**3)*ref_angl
        return angle, velocity
    else:
        return ref_angl, 0

save_data = []


if __name__ == "__main__":

    starting_time = 1.5 # startinPelvis_positiong time of the swinging action

# ──────────────────────────────────────────────────────────────────────
#   Load xml mujoco model
# ──────────────────────────────────────────────────────────────────────
    mujoco_xml_path = 'mjcf/Right_Ankle_Closed_Chain.xml'
    mj_model = mujoco.MjModel.from_xml_path(mujoco_xml_path)
    mj_data = mujoco.MjData(mj_model)
    time_step = mj_model.opt.timestep     

# ──────────────────────────────────────────────────────────────────────
#   Init Low-level PD impedence controller
# ──────────────────────────────────────────────────────────────────────
    Joint_Kp = np.diag([50, 50]) # N.m/rad
    Joint_Kd = np.diag([0.25, 0.25]) #N.ms/rad
    PDController= PD_Control(Joint_Kp, Joint_Kd) # init robot's joint PD controller

# ──────────────────────────────────────────────────────────────────────
#   Init states and observations
# ──────────────────────────────────────────────────────────────────────

    T = 0.8 # sec, init time at the beginning
    f_control = 1000 # Hz
    dt_control = 1/f_control #
    t_control = 0
    time_count = 0
    motor_torque = np.zeros(mj_model.nu)

    theta_m5_mujo_vel = 0
    theta_m5_jaco_vel = 0
    theta_m6_jaco_vel = 0
# ──────────────────────────────────────────────────────────────────────
#   start simulation
# ──────────────────────────────────────────────────────────────────────
    viewer = launch_passive(mj_model, mj_data)
    mj_model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_GRAVITY  # Disable gravity for passive viewer
    #while viewer.is_running():
    while mj_data.time <= 6:
        mujoco.mj_step(mj_model, mj_data) 
        simu_time = mj_data.time # simulation time 
        # ──────────────────────────────────────────────────────────────────────
        #   Get feedback joints' angles and velocity
        # ──────────────────────────────────────────────────────────────────────
        motor_pos = Get_Motor_Pos(mj_model, mj_data) # Get feedback motor position
        motor_vel = Get_Motor_Vel(mj_model, mj_data)

        ankle_pos = Get_Ankle_Pos(mj_model, mj_data)
        ankle_vel = Get_Ankle_Vel(mj_model, mj_data)

        if (simu_time >= starting_time): # start ZMP_preview control 

            time_count += 1

            if time_count >= (int)(dt_control/time_step):
                # ──────────────────────────────────────────────────────────────────────
                #   Start main control task
                # ──────────────────────────────────────────────────────────────────────

                theta_5 = mj_data.joint("Right_Ankle_Roll").qpos[0]
                theta_6 = mj_data.joint("Right_Ankle_Pitch").qpos[0]
                theta_m5 = mj_data.joint("Motor5_Joint").qpos[0]
                theta_m6 = mj_data.joint("Motor6_Joint").qpos[0]

                theta_5_vel = mj_data.joint("Right_Ankle_Roll").qvel[0]
                theta_6_vel = mj_data.joint("Right_Ankle_Pitch").qvel[0]
                theta_m5_mujo_vel = mj_data.joint("Motor5_Joint").qvel[0]
                theta_m6_mujo_vel = mj_data.joint("Motor6_Joint").qvel[0]

                theta_m_jaco_dot = ankle_kinematics.motor_velocities_from_foot(r=theta_5, p=theta_6, r_d=theta_5_vel, p_d=theta_6_vel, left=False)
                theta_m5_jaco_vel = theta_m_jaco_dot[0]
                theta_m6_jaco_vel = theta_m_jaco_dot[1]

                # ──────────────────────────────────────────────────────────────────────
                #   target ankle's roll and pitch angle 
                # ──────────────────────────────────────────────────────────────────────
                roll_cmd = np.deg2rad(15)*np.sin(2*np.pi*0.5*t_control)
                pitch_cmd = np.deg2rad(15)*np.sin(2*np.pi*0.5*t_control)

                # ──────────────────────────────────────────────────────────────────────
                #   solve IK to get motor cmd
                # ──────────────────────────────────────────────────────────────────────
                #right_motor_pos_cmd = ankle_kinematics.right_leg_motor_angles(roll_cmd, pitch_cmd)

                # Compute motor torque based on PD control

                #motor_torque = PDController.PD_Calculate(right_motor_pos_cmd, motor_pos, np.zeros((mj_model.nu,1)), dt_control)
                
                # timer
                t_control += dt_control
                time_count = 0

        else:
            motor_ref_anlges = np.zeros((mj_model.nu, 1))
            motor_torque = PDController.PD_Calculate(motor_ref_anlges, motor_pos, np.zeros((mj_model.nu,1)), dt_control)

        #mj_data.ctrl = np.array(motor_torque).reshape(1,-1) 
    
        viewer.sync()
        
        # save data for plotting
        data_point = np.hstack([t_control, theta_m5_mujo_vel, theta_m5_jaco_vel])
        save_data.append(data_point)

    viewer.close()

    data_array = np.vstack(save_data)  # shape (N, 5)
    np.save('jacobian_evaluate.npy', data_array)


    # #save the simulation data
    # with open('save_data.json', 'w') as file:
    #     json.dump(np.array(save_data).tolist(), file)
