"""Pure-functional frame conversions used by the MAVLink bridge.

The bridge consumes vision pose in ENU (REP-103, x-east, y-north, z-up,
body x-forward, y-left, z-up) and forwards it to ArduPilot in NED
(x-north, y-east, z-down, body x-forward, y-right, z-down).

We keep the math here as pure functions of (np.ndarray, ...) so it can
be exercised without a running ROS graph, and so the rotation logic is
not duplicated inline with MAVLink wire-format glue.

All quaternion arguments use scipy convention [x, y, z, w].
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import math
import numpy as np
from scipy.spatial.transform import Rotation as R


# Rotation that takes a vector expressed in ENU to NED.
# ENU = (east, north, up), NED = (north, east, down).
# Equivalently, swap x<->y and flip z.
R_NED_FROM_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)

# Rotation from FLU body (REP-103, x-forward, y-left, z-up) to FRD body
# (ArduPilot convention, x-forward, y-right, z-down).
R_FRD_FROM_FLU = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)


def position_enu_to_ned(p_enu: np.ndarray) -> np.ndarray:
    """Convert a 3-vector position from ENU to NED."""
    return R_NED_FROM_ENU @ np.asarray(p_enu, dtype=float).reshape(3)


def velocity_enu_to_ned(v_enu: np.ndarray) -> np.ndarray:
    """Convert a 3-vector linear velocity from ENU to NED."""
    return R_NED_FROM_ENU @ np.asarray(v_enu, dtype=float).reshape(3)


def quaternion_enu_flu_to_ned_frd(q_xyzw_enu_flu: np.ndarray) -> np.ndarray:
    """Convert orientation quaternion from ENU/FLU to NED/FRD.

    The input quaternion expresses the rotation that takes a vector from
    body-FLU to world-ENU. The output expresses body-FRD to world-NED.

    R_ned_frd = R_ned_enu * R_enu_flu * R_flu_frd

    where R_flu_frd = R_frd_flu^T.
    """
    R_enu_flu = R.from_quat(np.asarray(q_xyzw_enu_flu, dtype=float).reshape(4)).as_matrix()
    R_ned_frd = R_NED_FROM_ENU @ R_enu_flu @ R_FRD_FROM_FLU.T
    quat_xyzw = R.from_matrix(R_ned_frd).as_quat()
    return _normalize_quaternion_positive_w(np.asarray(quat_xyzw, dtype=float))


def quaternion_xyzw_to_wxyz(q_xyzw: np.ndarray) -> Tuple[float, float, float, float]:
    """Reorder a normalized scipy quaternion into MAVLink (w, x, y, z) order."""
    x, y, z, w = (float(value) for value in np.asarray(q_xyzw, dtype=float).reshape(4))
    return (w, x, y, z)


def world_to_body(rotation_world_body_xyzw: np.ndarray, v_world: np.ndarray) -> np.ndarray:
    """Rotate a vector expressed in the world frame into the body frame."""
    rot = R.from_quat(np.asarray(rotation_world_body_xyzw, dtype=float).reshape(4))
    return np.asarray(rot.inv().apply(np.asarray(v_world, dtype=float).reshape(3)), dtype=float)


def _normalize_quaternion_positive_w(q_xyzw: np.ndarray) -> np.ndarray:
    """Return the quaternion as a unit quaternion with non-negative w.

    ArduPilot's EKF dislikes both denormalized quaternions and the
    sign-ambiguous double-cover, so we canonicalize to the hemisphere
    with w >= 0.
    """
    q = np.asarray(q_xyzw, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    q = q / norm
    if q[3] < 0.0:
        q = -q
    return q


@dataclass(frozen=True)
class OdometryNed:
    """Pose+velocity in ArduPilot's expected MAV_FRAME_LOCAL_FRD/BODY_FRD frames."""

    position_ned: np.ndarray
    quaternion_wxyz: Tuple[float, float, float, float]
    velocity_body_frd: np.ndarray


def encode_odometry_ned(
    pos_enu: np.ndarray,
    quat_xyzw_enu_flu: np.ndarray,
    velocity_enu: np.ndarray,
) -> OdometryNed:
    """Convert an ENU/FLU vision pose+velocity into NED/FRD odometry.

    The position is reported in MAV_FRAME_LOCAL_FRD coordinates.
    The orientation is reported as the body-FRD to world-NED rotation,
    in MAVLink (w, x, y, z) order.
    The linear velocity is reported in body-FRD coordinates, since the
    MAVLink ODOMETRY message takes velocity in MAV_FRAME_BODY_FRD.
    """
    pos_ned = position_enu_to_ned(pos_enu)
    quat_xyzw_ned_frd = quaternion_enu_flu_to_ned_frd(quat_xyzw_enu_flu)
    quat_wxyz_ned_frd = quaternion_xyzw_to_wxyz(quat_xyzw_ned_frd)
    vel_ned = velocity_enu_to_ned(velocity_enu)
    vel_body_frd = world_to_body(quat_xyzw_ned_frd, vel_ned)
    return OdometryNed(
        position_ned=pos_ned,
        quaternion_wxyz=quat_wxyz_ned_frd,
        velocity_body_frd=np.asarray(vel_body_frd, dtype=float).reshape(3),
    )


def diagonal_pose_covariance(
    pos_std_m: float,
    rot_std_rad: float,
) -> Tuple[float, ...]:
    """Build a 21-element upper-triangular pose covariance with diagonal entries.

    MAVLink ODOMETRY's pose_covariance is the upper-right triangle of a
    6x6 matrix, ordered xx, xy, xz, xrr, xpp, xyy, yy, yz, ..., yawyaw.
    We populate only the diagonal terms; off-diagonals stay 0.
    The order along the diagonal of the 6x6 matrix is (x, y, z, roll, pitch, yaw).
    """
    pos_var = max(float(pos_std_m), 1e-6) ** 2
    rot_var = max(float(rot_std_rad), 1e-6) ** 2
    diagonal = [pos_var, pos_var, pos_var, rot_var, rot_var, rot_var]
    return tuple(_upper_triangular_from_diagonal(diagonal))


def diagonal_velocity_covariance(
    linear_std_m_s: float,
    angular_std_rad_s: float,
) -> Tuple[float, ...]:
    """Build a 21-element upper-triangular velocity covariance."""
    lin_var = max(float(linear_std_m_s), 1e-6) ** 2
    ang_var = max(float(angular_std_rad_s), 1e-6) ** 2
    diagonal = [lin_var, lin_var, lin_var, ang_var, ang_var, ang_var]
    return tuple(_upper_triangular_from_diagonal(diagonal))


def _upper_triangular_from_diagonal(diagonal):
    """Pack a 6x6 diagonal matrix into the 21-element upper-triangular row order
    that MAVLink uses for ODOMETRY pose/velocity covariance fields."""
    n = len(diagonal)
    out = []
    for row in range(n):
        for col in range(row, n):
            out.append(diagonal[row] if row == col else 0.0)
    return out
