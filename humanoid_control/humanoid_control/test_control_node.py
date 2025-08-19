#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from scipy.spatial.transform import Rotation as R
import math
import numpy as np

TARGET_JOINTS = [
    'Left_Hip_Pitch', 'Left_Hip_Roll', 'Left_Hip_Yaw',
    'Left_Knee', 'Left_Ankle_Roll', 'Left_Ankle_Pitch',
    'Right_Hip_Pitch', 'Right_Hip_Roll', 'Right_Hip_Yaw',
    'Right_Knee', 'Right_Ankle_Roll', 'Right_Ankle_Pitch',
]

class TestControl(Node):
    def __init__(self):
        super().__init__('state_subscriber')

        joint_topic = '/joint_states'
        imu_topic = '/imu_sensor_broadcaster/imu'

        self.joint_sub = self.create_subscription(JointState, joint_topic, self.joint_cb, 50)
        self.imu_sub   = self.create_subscription(Imu,        imu_topic,   self.imu_cb,   50)

        self.joint_pos = {}   # {name: position}
        self.joint_vel = {}   # {name: velocity}
        self.imu  = None

    def joint_cb(self, msg: JointState):
        has_vel = len(msg.velocity) == len(msg.name)
        for i, n in enumerate(msg.name):
            if n not in TARGET_JOINTS:
                continue
            self.joint_pos[n] = msg.position[i] if i < len(msg.position) else float('nan')
            self.joint_vel[n] = msg.velocity[i] if has_vel else float('nan')


    def imu_cb(self, msg: Imu):
        self.imu = msg
        orientation = msg.orientation
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        g_world = np.array([0.0, 0.0, -1.0])
        self.g_proj = r.inv().apply(g_world).astype(np.double)

def main():
    rclpy.init()
    node = TestControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()