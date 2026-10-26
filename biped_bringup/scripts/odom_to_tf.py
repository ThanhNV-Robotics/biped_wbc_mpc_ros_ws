#!/usr/bin/env python3
"""Broadcasts the simulated floating-base pose as TF: world -> link0_torso.

mujoco_ros2_control publishes the free joint state as nav_msgs/Odometry on
/simulator/floating_base_state (ground truth, world frame) but no TF. Used by sim_floating.launch.py
in place of the static world -> link0_torso transform of the hanging scene.
"""

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):

    def __init__(self):
        super().__init__('odom_to_tf')
        # The odometry frame_id is 'odom'; RViz (display.rviz) uses 'world'
        self.parent_frame = self.declare_parameter('parent_frame', 'world').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.declare_parameter('odom_topic', '/simulator/floating_base_state').value,
            self.on_odom, 10)

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTf())


if __name__ == '__main__':
    main()
