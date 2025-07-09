import numpy as np
from low_level_fcn import ankle_kinematics
import mujoco
import numpy as np
from mujoco.viewer import launch_passive
import json

class PD_Impedence_Control: # MIMO, joint position PD control
    def __init__(self, Kp, Kd):
        n = np.shape(Kp)[1]
        self.Kp = Kp
        self.Kd = Kd
        
    def PD_Control_Calculate (self, ref_pos, ref_vel, fb_pos, fb_vel): # MIMO
        # tqr_ff is the feedforward torque

        torque = self.Kp@(ref_pos - fb_pos) + self.Kd@(ref_vel - fb_vel)
        return torque # return torque vector applied to robot's joints

def Get_Motor_Pos (mj_model, mj_data):
    motor_5_joint_id = mj_model.joint("Motor5_Joint").id
    motor_6_joint_id = mj_model.joint("Motor6_Joint").id

    theta_m5 = mj_data.qpos[motor_5_joint_id]
    theta_m6 = mj_data.qpos[motor_6_joint_id]

    return np.array([theta_m5,theta_m6])

def Get_Motor_Vel (mj_model, mj_data):
    motor_5_joint_id = mj_model.joint("Motor5_Joint").id
    motor_6_joint_id = mj_model.joint("Motor6_Joint").id
    theta_m5_vel = mj_data.qvel[motor_5_joint_id]
    theta_m6_vel = mj_data.qvel[motor_6_joint_id]

    return np.array([theta_m5_vel,theta_m6_vel])

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
    PDController= PD_Impedence_Control(Joint_Kp, Joint_Kd) # init robot's joint PD controller

# ──────────────────────────────────────────────────────────────────────
#   Init states and observations
# ──────────────────────────────────────────────────────────────────────


    T = 0.8 # sec, init time at the beginning
    f_control = 500 # Hz
    dt_control = 1/f_control #
    t_control = 0
    time_count = 0

# ──────────────────────────────────────────────────────────────────────
#   target ankle's roll and pitch angle 
# ──────────────────────────────────────────────────────────────────────
    roll_cmd = np.deg2rad(20)
    pitch_cmd = np.deg2rad(20)
# ──────────────────────────────────────────────────────────────────────
#   solve IK to get motor cmd
# ──────────────────────────────────────────────────────────────────────
    right_motor_pos_cmd =  np.zeros(mj_model.nu) 
    motor_torque = np.zeros(mj_model.nu)

# ──────────────────────────────────────────────────────────────────────
#   start simulation
# ──────────────────────────────────────────────────────────────────────
    viewer = launch_passive(mj_model, mj_data)
    while viewer.is_running():
    #while mj_data.time <= 5:
        mujoco.mj_step(mj_model, mj_data) 
        simu_time = mj_data.time # simulation time 

        # ──────────────────────────────────────────────────────────────────────
        #   Get feedback joints' angles and velocity
        # ──────────────────────────────────────────────────────────────────────
        motor_pos_fb = Get_Motor_Pos(mj_model, mj_data) # Get feedback position
        motor_vel_fb = Get_Motor_Vel(mj_model, mj_data)

        if (simu_time >= starting_time): # start ZMP_preview control 
            # Get feedback CoM and ZMP Position
            time_count += 1

            if time_count >= (int)(dt_control/time_step):
                # ──────────────────────────────────────────────────────────────────────
                #   Start main control task
                # ──────────────────────────────────────────────────────────────────────
                # Generate motor angle trajectory
                ankle_kinematics.right_leg_motor_angles(roll_cmd, pitch_cmd)
                motor_ref_anlges, motor_ref_vel = step_ref(t_control, T, right_motor_pos_cmd)

                motor_torque = PDController.PD_Control_Calculate(motor_ref_anlges, motor_ref_vel, motor_pos_fb, motor_vel_fb)

                t_control += dt_control
                time_count = 0

        else:
            motor_ref_anlges, motor_ref_vel = 0, 0
            motor_torque = PDController.PD_Control_Calculate(motor_ref_anlges, motor_ref_vel, motor_pos_fb, motor_vel_fb)

        mj_data.ctrl = np.array(motor_torque).reshape(1,-1) 
    
        viewer.sync()

        # save_data.append([mj_data.time, CoM_vel_d[0].item(), CoM_vel_d[1].item(), CoM_vel_d[2].item(),\
        #                   CoM_feedback[0].item(), CoM_feedback[1].item(), CoM_feedback[2].item()])
        #save_data.append([mj_data.time, np.array(Robot_Control_System.robot_joint_ref_vel)])

    viewer.close()

    # save the simulation data
    # with open('save_data.json', 'w') as file:
    #     json.dump(save_data, file)   