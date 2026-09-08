"""Small rigid-transform helpers, free of any ROS dependency.

Everything the mission stack needs to move a point between frames lives here so
it can be unit tested without a running graph.  Transforms are 4x4 homogeneous
matrices, quaternions are (x, y, z, w) to match geometry_msgs.
"""

import math

import numpy as np


def quaternion_matrix(x, y, z, w):
    """4x4 rotation matrix from a geometry_msgs-ordered quaternion."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(4)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def matrix_quaternion(m):
    """geometry_msgs-ordered quaternion from a 4x4 (or 3x3) rotation matrix."""
    r = np.asarray(m)[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def transform_matrix(translation, quaternion):
    """4x4 matrix from a (x, y, z) translation and an (x, y, z, w) quaternion."""
    m = quaternion_matrix(*quaternion)
    m[0, 3], m[1, 3], m[2, 3] = translation
    return m


def matrix_from_transform_msg(transform):
    """4x4 matrix from a geometry_msgs/Transform (or the .transform of a stamped one)."""
    t = transform.translation
    q = transform.rotation
    return transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def matrix_from_pose_msg(pose):
    """4x4 matrix from a geometry_msgs/Pose."""
    p = pose.position
    q = pose.orientation
    return transform_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def invert(m):
    """Inverse of a rigid 4x4 transform, without a general matrix inversion."""
    r = np.asarray(m)[:3, :3]
    t = np.asarray(m)[:3, 3]
    out = np.eye(4)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ t
    return out


def translation_of(m):
    """(x, y, z) of a 4x4 transform."""
    m = np.asarray(m)
    return (float(m[0, 3]), float(m[1, 3]), float(m[2, 3]))


def yaw_of(m):
    """Rotation about z of a 4x4 transform, radians."""
    m = np.asarray(m)
    return math.atan2(float(m[1, 0]), float(m[0, 0]))


def yaw_quaternion(yaw):
    """(x, y, z, w) for a rotation of `yaw` radians about z."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quaternion_yaw(x, y, z, w):
    """Yaw of a geometry_msgs-ordered quaternion, radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# --------------------------------------------------------------------------- #
# Camera optical convention
# --------------------------------------------------------------------------- #

def optical_rotation():
    """Body-aligned (x forward, y left, z up) to optical (z forward, x right, y down).

    The burger SDF omits `frame_name` on the Gazebo camera plugin, so images are
    stamped with the *link* frame `camera_rgb_frame`, which is body aligned.
    apriltag_ros nonetheless solves the tag pose in the optical convention every
    calibrated camera uses, and publishes it as a child of that same body-aligned
    frame.  The numbers are therefore correct but expressed in the wrong basis:
    tag IDs come out right and positions come out rotated.

    Left-multiplying the reported camera->tag transform by this matrix puts the
    measurement back into the body-aligned frame it was stamped in.  On hardware
    the driver stamps a real optical frame, so the correction is switched off
    there and this reduces to the identity.
    """
    m = np.eye(4)
    m[:3, :3] = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    return m


def planar_distance(a, b):
    """Distance in the xy plane between two (x, y, ...) sequences."""
    return math.hypot(a[0] - b[0], a[1] - b[1])
