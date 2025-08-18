#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class TrajectoryPublisherNode(Node):
    def __init__(self):
        super().__init__('trajectory_100hz_tester')

        # 질문에 주신 관절 리스트
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

        self.freq_hz = 0.5       # 사인파 주파수 [Hz]
        self.amp_rad = math.radians(10.0)  # 진폭 [rad] (±10도)
        self.total_time = 5.0    # 전체 궤적 시간 [s]
        self.send_rate = 100.0   # 샘플링/전송 간격 [Hz] → 100 Hz

        self.ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for follow_joint_trajectory action server...')
        self.ac.wait_for_server()

        self.get_logger().info('Action server available. Building and sending 100 Hz trajectory...')
        self._send_trajectory()

    def _build_trajectory(self) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        dt = 1.0 / self.send_rate
        steps = int(self.total_time * self.send_rate)

        # Left_Hip_Pitch 인덱스
        left_hip_pitch_idx = self.joint_names.index('Left_Hip_Pitch')
        right_hip_pitch_idx = self.joint_names.index('Right_Hip_Pitch')

        t = 0.0
        for i in range(steps + 1):
            p = [0.0] * len(self.joint_names)
            p[left_hip_pitch_idx] = self.amp_rad * math.sin(2.0 * math.pi * self.freq_hz * t)
            p[right_hip_pitch_idx] = self.amp_rad * math.sin(2.0 * math.pi * self.freq_hz * t + math.pi)

            pt = JointTrajectoryPoint()
            pt.positions = p
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            traj.points.append(pt)

            t += dt

        return traj

    def _send_trajectory(self):
        traj = self._build_trajectory()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=0, nanosec=200_000_000)

        send_goal_future = self.ac.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by controller.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Controller result received: error_code={result.error_code}, error_string="{result.error_string}"')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()