#!/usr/bin/env python3
import math
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PDEffortTrajectoryNode(Node):
    """
    Gazebo(ros_gz_sim) + forward_command_controller(interface=effort) 조합 전용.
    - /joint_states 구독 → q, dq 취득
    - 내부 궤적 생성 q_d(t), dq_d(t)
    - PD: tau = Kp*(q_d - q) + Kd*(dq_d - dq)
    - /effort_cmd/commands 로 Float64MultiArray 퍼블리시
    """

    def __init__(self):
        super().__init__('pd_effort_trajectory')

        self.joint_names: List[str] = [
            'Left_Hip_Pitch',
            'Left_Hip_Roll',
            'Left_Hip_Yaw',
            'Left_Knee',
            'Left_Ankle_Roll',
            'Left_Ankle_Pitch',
            'Right_Hip_Pitch',
            'Right_Hip_Roll',
            'Right_Hip_Yaw',
            'Right_Knee',
            'Right_Ankle_Roll',
            'Right_Ankle_Pitch',
        ]

        self.rate_hz: float = 100.0
        self.topic_effort: str = "/effort_cmd/commands"

        self.Kp = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Kd = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.torque_lim = [60.0, 40.0, 30.0, 60.0, 20.0, 20.0, 60.0, 40.0, 30.0, 60.0, 20.0, 20.0]

        self.amp_rad = math.radians(15.0)
        self.freq_hz = 1.0
        self.use_sine_hips_only = True

        # ===== 상태 =====
        self.q = [0.0] * len(self.joint_names)
        self.dq = [0.0] * len(self.joint_names)
        self.have_state = False
        self.name_to_index: Dict[str, int] = {n: i for i, n in enumerate(self.joint_names)}

        self.idx_LHP = self.name_to_index.get('Left_Hip_Pitch', None)
        self.idx_RHP = self.name_to_index.get('Right_Hip_Pitch', None)

        self.sub_js = self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 50)
        self.pub_effort = self.create_publisher(Float64MultiArray, self.topic_effort, 10)
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.control_step)

        self.t = 0.0
        self.get_logger().info(f'PD Effort Controller running at {self.rate_hz:.1f} Hz, publishing to {self.topic_effort}')

    def cb_joint_states(self, msg: JointState):
        name_to_idx_js = {n: i for i, n in enumerate(msg.name)}
        for jn, jidx in self.name_to_index.items():
            if jn in name_to_idx_js:
                i = name_to_idx_js[jn]
                if i < len(msg.position): self.q[jidx] = msg.position[i]
                if i < len(msg.velocity): self.dq[jidx] = msg.velocity[i]
        self.have_state = True

    def desired_traj(self, t: float):
        qd = [0.0] * len(self.joint_names)
        dqd = [0.0] * len(self.joint_names)

        if self.use_sine_hips_only and self.idx_LHP is not None and self.idx_RHP is not None:
            w = 2.0 * math.pi * self.freq_hz
            qd[self.idx_LHP] = self.amp_rad * math.sin(w * t)
            dqd[self.idx_LHP] = self.amp_rad * w * math.cos(w * t)

            qd[self.idx_RHP] = self.amp_rad * math.sin(w * t + math.pi)
            dqd[self.idx_RHP] = self.amp_rad * w * math.cos(w * t + math.pi)

        else:
            w = 2.0 * math.pi * self.freq_hz
            val = self.amp_rad * math.sin(w * t)
            dval = self.amp_rad * w * math.cos(w * t)
            for i in range(len(self.joint_names)):
                qd[i] = val
                dqd[i] = dval
        return qd, dqd

    # ─────────────────────────────────────────
    # 제어 스텝: PD → 토크 → publish
    # ─────────────────────────────────────────
    def control_step(self):
        if not self.have_state:
            return

        qd, dqd = self.desired_traj(self.t)

        tau = [0.0] * len(self.joint_names)
        for i in range(len(self.joint_names)):
            e = qd[i] - self.q[i]
            ed = dqd[i] - self.dq[i]
            tau_i = self.Kp[i] * e + self.Kd[i] * ed
            lim = self.torque_lim[i]
            if   tau_i >  lim: tau_i =  lim
            elif tau_i < -lim: tau_i = -lim
            tau[i] = tau_i

        msg = Float64MultiArray()
        msg.data = tau
        self.pub_effort.publish(msg)

        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = PDEffortTrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
