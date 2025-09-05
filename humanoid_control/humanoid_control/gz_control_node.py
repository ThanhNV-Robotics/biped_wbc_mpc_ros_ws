#!/usr/bin/env python3
from collections import deque
import math
import os
from typing import List, Dict
from dataclasses import dataclass

import torch

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

import numpy as np
from scipy.spatial.transform import Rotation as R

class cmd:
    vx = 0.0
    vy = 0.0
    dyaw = 0.0

@dataclass
class cfg:
    class env:
        frame_stack = 5
        num_single_obs = 45
        num_observations = int(frame_stack * num_single_obs)
        num_actions = 12

    class normalization:
        class obs_scales:
            # lin_vel = 2.
            ang_vel = 0.2
            dof_pos = 1.0
            dof_vel = 0.05
        clip_observations = 18.
        clip_actions = 18.

    class control():
        stiffness = {
            "hip_pitch": 150.0,
            "hip_roll": 150.0,
            "hip_yaw":  100.0,
            "knee": 150.0,     
            "ankle": 20.0,
        }

        damping = {
            "hip_pitch": 1,
            "hip_roll": 1,
            "hip_yaw":1,
            "knee": 1,     
            "ankle": 1,
        }
        
        action_scale = 0.25

    class sim_config:
        dt = 0.002
        decimation = 5

TARGET_JOINTS = [
    'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
    'left_knee', 'left_ankle_roll', 'left_ankle_pitch',
    'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
    'right_knee', 'right_ankle_roll', 'right_ankle_pitch',
]

def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd

def quaternion_to_euler(quat):
    x, y, z, w = quat

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    euler = np.array([roll, pitch, yaw])
    euler[euler >  math.pi]  -= 2*math.pi
    euler[euler < -math.pi]  += 2*math.pi
    return euler

class gzControlNode(Node):
    """
    Gazebo(ros_gz_sim) + forward_command_controller(interface=effort) 조합 전용.
    - /joint_states 구독 → q, dq 취득
    - 내부 궤적 생성 q_d(t), dq_d(t)
    - PD: tau = Kp*(q_d - q) + Kd*(dq_d - dq)
    """
    def __init__(self):
        super().__init__('gz_control_node')

        package_name = 'humanoid_control'
        package_share_directory = get_package_share_directory(package_name)
        model_path = os.path.join(package_share_directory, 'model', 'policy_stand.pt')
        self.policy = torch.jit.load(model_path)
        
        joint_topic = '/joint_states'
        imu_topic = '/imu_sensor_broadcaster/imu'
        effort_topic = '/effort_cmd/commands'

        self.joint_sub = self.create_subscription(JointState, joint_topic, self._cb_joint, 50)
        self.imu_sub   = self.create_subscription(Imu,        imu_topic,   self._cb_imu,   50)

        self.effort_pub = self.create_publisher(Float64MultiArray, effort_topic, 10)

        self.q = np.zeros(len(TARGET_JOINTS), dtype=np.float32)
        self.dq = np.zeros(len(TARGET_JOINTS), dtype=np.float32)

        self.target_q = np.zeros(cfg.env.num_actions, dtype=np.float32)
        self.action = np.zeros(cfg.env.num_actions, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.base_euler_xyz = np.zeros(3, dtype=np.float32)

        self.Kp = np.zeros(len(TARGET_JOINTS), dtype=np.float32)
        self.Kd = np.zeros(len(TARGET_JOINTS), dtype=np.float32)
        self.tau_limit = np.array([100.0] * len(TARGET_JOINTS), dtype=np.float32)

        for i, name in enumerate(TARGET_JOINTS):
            for base_name in cfg.control.stiffness.keys():
                if base_name in name:
                    self.Kp[i] = cfg.control.stiffness[base_name]
                    self.Kd[i] = cfg.control.damping[base_name]
                    break

        self.hist_obs = deque()
        for _ in range(cfg.env.frame_stack):
            self.hist_obs.append(np.zeros([1, cfg.env.num_single_obs], dtype=np.double))

        self.cycle_time = 1.1

        self.name_to_index: Dict[str, int] = {n: i for i, n in enumerate(TARGET_JOINTS)}

        self.count_lowlevel = 0
        self.timer = self.create_timer(cfg.sim_config.dt, self._control)
    
    def _cb_joint(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.name_to_index:
                idx = self.name_to_index[name]
                self.q[idx] = msg.position[i]
                self.dq[idx] = msg.velocity[i]

    def _cb_imu(self, msg: Imu):
        self.imu = msg
        self.base_ang_vel = np.array([self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z])
        # self.base_euler_xyz = quaternion_to_euler([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
        r = R.from_quat([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
        g_world = np.array([0.0, 0.0, -1.0])
        self.g_proj = r.inv().apply(g_world).astype(np.double)


    def _get_obs(self):
        obs = np.zeros([1, cfg.env.num_single_obs], dtype=np.float32)

        # obs[0, 0] = math.sin(2*math.pi * self.count_lowlevel * cfg.sim_config.dt / self.cycle_time)
        # obs[0, 1] = math.cos(2*math.pi * self.count_lowlevel * cfg.sim_config.dt / self.cycle_time)
        # obs[0, 2] = cmd.vx * cfg.normalization.obs_scales.lin_vel
        # obs[0, 3] = cmd.vy * cfg.normalization.obs_scales.lin_vel
        # obs[0, 4] = cmd.dyaw * cfg.normalization.obs_scales.ang_vel
        # obs[0, 5:17] = self.q * cfg.normalization.obs_scales.dof_pos
        # obs[0, 17:29] = self.dq * cfg.normalization.obs_scales.dof_vel
        # obs[0, 29:41] = self.action
        # obs[0, 41:44] = self.base_ang_vel
        # obs[0, 44:47] = self.base_euler_xyz

        obs[0, :3] = self.base_ang_vel * cfg.normalization.obs_scales.ang_vel
        obs[0, 3:6] = self.g_proj
        obs[0, 6] = cmd.vx
        obs[0, 7] = cmd.vy
        obs[0, 8] = cmd.dyaw
        obs[0, 9:21] = self.q * cfg.normalization.obs_scales.dof_pos
        obs[0, 21:33] = self.dq * cfg.normalization.obs_scales.dof_vel
        obs[0, 33:45] = self.action

        obs = np.clip(obs, -cfg.normalization.clip_observations, cfg.normalization.clip_observations)
        return obs
    
    def _control(self):
        if self.count_lowlevel % cfg.sim_config.decimation == 0:
            observation = self._get_obs()

            self.hist_obs.append(observation)
            self.hist_obs.popleft()

            obs_array = np.squeeze(np.array(self.hist_obs))

            feature_groups = {
                'base_ang_vel': (0, 3),
                'g_proj':       (3, 6),
                'commands':     (6, 9),
                'dof_pos':      (9, 21),
                'dof_vel':      (21, 33),
                'last_action':  (33, 45)
            }
            
            policy_input_chunks = []
            
            for group_name in feature_groups:
                start, end = feature_groups[group_name]
                chunk_history = obs_array[:, start:end] 
                flattened_chunk = chunk_history.flatten()
                policy_input_chunks.append(flattened_chunk)
            
            policy_input = np.concatenate(policy_input_chunks).reshape(1, -1)
            
            action = self.policy(torch.tensor(policy_input.astype(np.float32)))[0].detach().numpy()
            action = np.clip(action, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)

            self.target_q = action * cfg.control.action_scale
            self.action = action.astype(np.float32)

        target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
        tau = pd_control(self.target_q, self.q, self.Kp, target_dq, self.dq, self.Kd)
        tau = np.clip(tau, -self.tau_limit, self.tau_limit)

        self.effort_pub.publish(Float64MultiArray(data=tau))
        self.count_lowlevel += 1

def main(args=None):
    rclpy.init(args=args)
    node = gzControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
