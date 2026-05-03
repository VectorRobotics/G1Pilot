"""
Generic pose filtering node with moving-average and outlier rejection.

Supports PoseStamped and Pose2D message types. Publishes filtered output on a
configurable topic (defaults to {input_topic}_filtered). Designed to be launched
multiple times with different configurations.

For PoseStamped:
  - Position filtered via per-axis moving average with std-based outlier rejection
  - Orientation filtered via eigenvector-averaged quaternion (Markley method)
    with angular-distance outlier rejection
  - Optional TF transform to a target frame before filtering

For Pose2D:
  - x/y filtered via moving average with std-based outlier rejection
  - theta filtered via circular mean with angular-distance outlier rejection
"""

from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion

from humanoid_arm_skills.utils import (
    average_quaternions,
    quaternion_angular_distance,
    normalize_angle,
    upright_pose,
)

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform


class PoseFilterNode(Node):
    """Generic pose filter with moving-average and outlier rejection."""

    def __init__(self):
        super().__init__('pose_filter')

        # Static parameters
        input_topic = self.declare_parameter('input_topic', '/pose_in').value
        output_topic = self.declare_parameter('output_topic', '/pose_out').value
        msg_type = self.declare_parameter('msg_type', 'PoseStamped').value
        self.target_frame = self.declare_parameter('target_frame_arm', 'pelvis').value

        # Dynamic parameters
        self.declare_parameter('window_size', 10)
        self.declare_parameter('position_outlier_std', 2.0)
        self.declare_parameter('orientation_outlier_rad', 0.5)
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('tf_timeout', 0.5)
        self.declare_parameter('lost_timeout', 5.0)

        window = self.get_parameter("window_size").value

        # State
        self.position_buffer = deque(maxlen=window)
        self.orientation_buffer = deque(maxlen=window)
        self.last_input_time = None
        self.is_3d = (msg_type == 'PoseStamped')

        # TF2 (only if needed)
        self.tf_buffer = None
        if self.is_3d and self.target_frame:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sub / pub
        if self.is_3d:
            self.create_subscription(
                PoseStamped, input_topic, self._pose_stamped_cb, 5)
            self.pub = self.create_publisher(PoseStamped, output_topic, 5)
            self.upright_pub = self.create_publisher(
                PoseStamped, f'{output_topic}_upright', 5)
        else:
            self.create_subscription(
                Pose2D, input_topic, self._pose2d_cb, 5)
            self.pub = self.create_publisher(Pose2D, output_topic, 5)

        # Lost timeout timer
        self.create_timer(0.1, self._check_timeout)

        self.get_logger().debug(
            f'Pose filter: {input_topic} ({msg_type}) -> {output_topic}'
            + (f' [tf -> {self.target_frame}]' if self.target_frame else '')
        )
        self.get_logger().info(
            '\033[32m\033[1m\u2714\033[0m pose_filtering: ready')

    # ── PoseStamped ──

    def _pose_stamped_cb(self, msg: PoseStamped):
        # Optional TF transform
        if self.tf_buffer and self.target_frame:
            try:
                msg.header.stamp = Time(seconds=0).to_msg()
                msg = self.tf_buffer.transform(
                    msg, self.target_frame,
                    timeout=Duration(seconds=self.get_parameter("tf_timeout").value))
            except tf2_ros.TransformException as e:
                self.get_logger().warn(
                    f'TF: {e}', throttle_duration_sec=2.0)
                return

        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        # Always add to buffer so window adapts over time
        self.position_buffer.append(pos)
        self.orientation_buffer.append(quat)
        self.last_input_time = self.get_clock().now().nanoseconds

        if len(self.position_buffer) < self.get_parameter("min_samples").value:
            return

        # Filter inliers for the average
        positions = np.array(self.position_buffer)
        quats = np.array(self.orientation_buffer)

        # Position: keep samples within std threshold
        pos_mean = np.mean(positions, axis=0)
        pos_std = np.maximum(np.std(positions, axis=0), 1e-6)
        pos_mask = np.all(
            np.abs(positions - pos_mean)
            <= self.get_parameter("position_outlier_std").value * pos_std,
            axis=1)

        # Orientation: keep samples within angular threshold
        avg_q = average_quaternions(quats)
        ori_mask = np.array([
            quaternion_angular_distance(
                Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                Quaternion(x=avg_q[0], y=avg_q[1], z=avg_q[2], w=avg_q[3]),
            ) <= self.get_parameter("orientation_outlier_rad").value
            for q in quats
        ])

        inlier_mask = pos_mask & ori_mask
        if np.sum(inlier_mask) < self.get_parameter("min_samples").value:
            return

        filt_pos = np.mean(positions[inlier_mask], axis=0)
        filt_quat = average_quaternions(quats[inlier_mask])

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.pose.position.x = float(filt_pos[0])
        out.pose.position.y = float(filt_pos[1])
        out.pose.position.z = float(filt_pos[2])
        out.pose.orientation = Quaternion(
            x=float(filt_quat[0]), y=float(filt_quat[1]),
            z=float(filt_quat[2]), w=float(filt_quat[3]))
        self.pub.publish(out)

        upright_out = PoseStamped()
        upright_out.header = out.header
        upright_out.pose = upright_pose(out.pose, flip=True)
        self.upright_pub.publish(upright_out)

    # ── Pose2D ──

    def _pose2d_cb(self, msg: Pose2D):
        pos = np.array([msg.x, msg.y])
        theta = msg.theta

        # Always add to buffer so window adapts over time
        self.position_buffer.append(pos)
        self.orientation_buffer.append(theta)
        self.last_input_time = self.get_clock().now().nanoseconds

        if len(self.position_buffer) < self.get_parameter("min_samples").value:
            return

        # Filter inliers for the average
        positions = np.array(self.position_buffer)
        angles = np.array(self.orientation_buffer)

        # Position: keep samples within std threshold
        pos_mean = np.mean(positions, axis=0)
        pos_std = np.maximum(np.std(positions, axis=0), 1e-6)
        pos_mask = np.all(
            np.abs(positions - pos_mean)
            <= self.get_parameter("position_outlier_std").value * pos_std,
            axis=1)

        # Angle: keep samples within angular threshold (circular)
        mean_angle = np.arctan2(
            np.mean(np.sin(angles)), np.mean(np.cos(angles)))
        ori_mask = np.array([
            abs(normalize_angle(a - mean_angle))
            <= self.get_parameter("orientation_outlier_rad").value
            for a in angles
        ])

        inlier_mask = pos_mask & ori_mask
        if np.sum(inlier_mask) < self.get_parameter("min_samples").value:
            return

        filt_pos = np.mean(positions[inlier_mask], axis=0)
        inlier_angles = angles[inlier_mask]
        filt_theta = np.arctan2(
            np.mean(np.sin(inlier_angles)),
            np.mean(np.cos(inlier_angles)))

        out = Pose2D()
        out.x = float(filt_pos[0])
        out.y = float(filt_pos[1])
        out.theta = float(filt_theta)
        self.pub.publish(out)

    # ── Timeout ──

    def _check_timeout(self):
        if self.last_input_time is None:
            return
        elapsed = (self.get_clock().now().nanoseconds
                   - self.last_input_time) * 1e-9
        if elapsed > self.get_parameter("lost_timeout").value:
            self.position_buffer.clear()
            self.orientation_buffer.clear()
            self.last_input_time = None


def main(args=None):
    rclpy.init(args=args)
    node = PoseFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
