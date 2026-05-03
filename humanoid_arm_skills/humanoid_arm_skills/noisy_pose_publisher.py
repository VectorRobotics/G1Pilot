#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


class NoisyPosePublisher(Node):
    def __init__(self):
        super().__init__('noisy_pose_publisher')

        self.declare_parameter('topic', 'track_3d/selected_normal_pose')
        self.declare_parameter('frame_id', 'pelvis')
        self.declare_parameter('rate', 10.0)

        # Mean pose
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 1.0)
        self.declare_parameter('qw', 0.0)

        # Noise std devs
        self.declare_parameter('pos_noise_std', 0.01)
        self.declare_parameter('rot_noise_std', 0.01)

        topic = self.get_parameter('topic').value
        rate = self.get_parameter('rate').value

        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish)

    def publish(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        pos_std = self.get_parameter('pos_noise_std').value
        rot_std = self.get_parameter('rot_noise_std').value

        msg.pose.position.x = self.get_parameter('x').value + np.random.normal(0, pos_std)
        msg.pose.position.y = self.get_parameter('y').value + np.random.normal(0, pos_std)
        msg.pose.position.z = self.get_parameter('z').value + np.random.normal(0, pos_std)

        qx = self.get_parameter('qx').value + np.random.normal(0, rot_std)
        qy = self.get_parameter('qy').value + np.random.normal(0, rot_std)
        qz = self.get_parameter('qz').value + np.random.normal(0, rot_std)
        qw = self.get_parameter('qw').value + np.random.normal(0, rot_std)
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        msg.pose.orientation.x = qx / norm
        msg.pose.orientation.y = qy / norm
        msg.pose.orientation.z = qz / norm
        msg.pose.orientation.w = qw / norm

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NoisyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
