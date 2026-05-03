"""Local replacements for the few vector_utils.transform_utils helpers used here."""

import math

import numpy as np

from geometry_msgs.msg import Pose, Quaternion


def normalize_angle(angle):
    """Normalize angle (rad) to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def quaternion_angular_distance(q1: Quaternion, q2: Quaternion) -> float:
    """Angular distance in radians between two quaternions, in [0, pi]."""
    dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    return 2.0 * np.arccos(np.clip(abs(dot), 0.0, 1.0))


def pose_distance(p1: Pose, p2: Pose) -> tuple[float, float]:
    """Position (metres) and orientation (degrees) distance between two poses."""
    dp = math.sqrt(
        (p1.position.x - p2.position.x) ** 2
        + (p1.position.y - p2.position.y) ** 2
        + (p1.position.z - p2.position.z) ** 2)
    d_ori_deg = math.degrees(quaternion_angular_distance(
        p1.orientation, p2.orientation))
    return dp, d_ori_deg


def average_quaternions(quaternions: np.ndarray) -> np.ndarray:
    """Average quaternion via the eigenvector method (Markley et al.).

    Input/output use [x, y, z, w] order. Handles q/-q sign ambiguity.
    """
    Q = np.asarray(quaternions, dtype=np.float64)
    if Q.ndim == 1:
        return Q / np.linalg.norm(Q)

    for i in range(1, len(Q)):
        if np.dot(Q[i], Q[0]) < 0.0:
            Q[i] = -Q[i]

    M = Q.T @ Q
    eigenvalues, eigenvectors = np.linalg.eigh(M)
    avg = eigenvectors[:, eigenvalues.argmax()]
    return avg / np.linalg.norm(avg)


def upright_pose(pose: Pose, flip: bool = False) -> Pose:
    """Return a pose with the same position but a yaw-only (Z-up) orientation.

    The yaw is taken from the projection of the input pose's +X axis onto the
    XY plane. If ``flip`` is True, yaw is rotated by 180°.
    """
    x = pose.orientation.x
    y = pose.orientation.y
    z = pose.orientation.z
    w = pose.orientation.w
    if (x * x + y * y + z * z + w * w) == 0.0:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_y = 2.0 * (x * y + w * z)
    yaw = float(np.arctan2(forward_y, forward_x))
    if flip:
        yaw += np.pi

    result = Pose()
    result.position = pose.position
    result.orientation = Quaternion(
        x=0.0, y=0.0,
        z=float(np.sin(yaw / 2)),
        w=float(np.cos(yaw / 2)),
    )
    return result
