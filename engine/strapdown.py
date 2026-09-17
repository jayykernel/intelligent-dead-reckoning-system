"""
Classical Strapdown Inertial Navigation System (INS) Mechanization.

Implements pure physics-based numerical integration in a local East-North-Up (ENU)
navigation frame. No learned models, no zero-velocity updates (ZUPT), and no
non-holonomic constraints (NHC) — serving as the unconstrained baseline.
"""

from typing import Dict, Tuple, Optional, Union
import numpy as np


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product q = q1 * q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ], dtype=np.float64)


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """
    Convert unit quaternion q = [qw, qx, qy, qz] to Direction Cosine Matrix R_b^n.
    Transforms vectors from body frame to navigation frame: v_n = R_b^n * v_b.
    """
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = q / norm
    return np.array([
        [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z),       1.0 - 2.0 * (x**2 + z**2), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x**2 + y**2)]
    ], dtype=np.float64)


def rotvec_to_quat(rot_vec: np.ndarray) -> np.ndarray:
    """Convert rotation vector (axis * angle) to delta quaternion."""
    angle = float(np.linalg.norm(rot_vec))
    if angle < 1e-12:
        return np.array([1.0 - angle**2 / 8.0, rot_vec[0] / 2.0, rot_vec[1] / 2.0, rot_vec[2] / 2.0], dtype=np.float64)
    axis = rot_vec / angle
    half_angle = angle / 2.0
    sin_half = np.sin(half_angle)
    return np.array([np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half], dtype=np.float64)


def dcm_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix R to unit quaternion [qw, qx, qy, qz]."""
    tr = np.trace(R)
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    return q / np.linalg.norm(q)


class StrapdownINS:
    """
    Classical 6-DOF Strapdown Inertial Navigation Mechanization in ENU frame.

    State:
        position: [East, North, Up] in meters
        velocity: [v_East, v_North, v_Up] in m/s
        quaternion: [qw, qx, qy, qz] representing Body -> ENU rotation
    """

    def __init__(self, gravity: float = 9.80665):
        self.g = float(gravity)
        self.g_n = np.array([0.0, 0.0, -self.g], dtype=np.float64)
        self.pos = np.zeros(3, dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def initialize(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        q0: Optional[np.ndarray] = None,
        R0: Optional[np.ndarray] = None
    ) -> None:
        """Initialize position, velocity, and orientation."""
        self.pos = np.array(p0, dtype=np.float64).copy()
        self.vel = np.array(v0, dtype=np.float64).copy()
        if q0 is not None:
            self.quat = np.array(q0, dtype=np.float64) / np.linalg.norm(q0)
        elif R0 is not None:
            self.quat = dcm_to_quat(np.array(R0, dtype=np.float64))
        else:
            self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def initialize_from_gravity_and_heading(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        acc_init: np.ndarray,
        heading_init_deg: float
    ) -> None:
        """
        Coarse alignment / leveling:
        1. Estimates initial pitch and roll from the initial specific force vector (leveling).
        2. Aligns heading with initial bearing (deg clockwise from True North).
        """
        f0 = np.array(acc_init, dtype=np.float64)
        f_norm = np.linalg.norm(f0)
        if f_norm < 1e-3:
            u_b = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            u_b = f0 / f_norm

        u_n = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # Up in ENU

        # Axis-angle rotation from u_b to u_n
        v_axis = np.cross(u_b, u_n)
        s_angle = np.linalg.norm(v_axis)
        c_angle = np.dot(u_b, u_n)
        if s_angle > 1e-6:
            vx = np.array([
                [0.0, -v_axis[2], v_axis[1]],
                [v_axis[2], 0.0, -v_axis[0]],
                [-v_axis[1], v_axis[0], 0.0]
            ])
            R_level = np.eye(3) + vx + vx @ vx * ((1.0 - c_angle) / (s_angle**2))
        else:
            R_level = np.eye(3)

        # Yaw rotation in ENU (East-North plane)
        # Heading psi is clockwise from North:
        # Forward in ENU is [sin(psi), cos(psi), 0]
        # East in ENU is [cos(psi), -sin(psi), 0]
        psi = np.radians(heading_init_deg)
        R_yaw = np.array([
            [np.cos(psi), np.sin(psi), 0.0],
            [-np.sin(psi), np.cos(psi), 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        R0 = R_yaw @ R_level
        self.initialize(p0, v0, R0=R0)

    def step(self, acc_b: np.ndarray, gyro_b: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Execute one discrete strapdown mechanization step.

        Args:
            acc_b: Specific force in body frame [ax, ay, az] (m/s^2)
            gyro_b: Angular velocity in body frame [wx, wy, wz] (rad/s)
            dt: Time interval in seconds

        Returns:
            Tuple of (pos, vel, quat) in ENU navigation frame
        """
        # 1. Attitude Update via Quaternion
        rot_vec = np.array(gyro_b, dtype=np.float64) * dt
        dq = rotvec_to_quat(rot_vec)
        self.quat = quat_mult(self.quat, dq)
        self.quat = self.quat / np.linalg.norm(self.quat)

        # 2. Specific Force Transformation to ENU Frame
        R_b_n = quat_to_dcm(self.quat)
        f_n = R_b_n @ np.array(acc_b, dtype=np.float64)

        # 3. Acceleration in ENU Frame (specific force + gravity)
        a_n = f_n + self.g_n

        # 4. Velocity and Position Integration
        self.pos = self.pos + self.vel * dt + 0.5 * a_n * (dt**2)
        self.vel = self.vel + a_n * dt

        return self.pos.copy(), self.vel.copy(), self.quat.copy()

    def get_state(self) -> Dict[str, np.ndarray]:
        """Get current state dictionary."""
        R_b_n = quat_to_dcm(self.quat)
        return {
            "position": self.pos.copy(),
            "velocity": self.vel.copy(),
            "quaternion": self.quat.copy(),
            "rotation_matrix": R_b_n
        }

    def run_trajectory(
        self,
        acc_array: np.ndarray,
        gyro_array: np.ndarray,
        dt: float,
        p0: np.ndarray,
        v0: np.ndarray,
        q0: Optional[np.ndarray] = None,
        R0: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Run batch strapdown integration over entire trajectory arrays.

        Args:
            acc_array: (N, 3) specific force in body frame
            gyro_array: (N, 3) angular rate in body frame
            dt: time step in seconds
            p0: (3,) initial position
            v0: (3,) initial velocity
            q0 or R0: initial attitude

        Returns:
            pos_hist: (N, 3) position trajectory
            vel_hist: (N, 3) velocity trajectory
            quat_hist: (N, 4) quaternion trajectory
        """
        N = len(acc_array)
        self.initialize(p0, v0, q0=q0, R0=R0)

        pos_hist = np.zeros((N, 3), dtype=np.float64)
        vel_hist = np.zeros((N, 3), dtype=np.float64)
        quat_hist = np.zeros((N, 4), dtype=np.float64)

        pos_hist[0] = self.pos
        vel_hist[0] = self.vel
        quat_hist[0] = self.quat

        for k in range(N - 1):
            p, v, q = self.step(acc_array[k], gyro_array[k], dt)
            pos_hist[k + 1] = p
            vel_hist[k + 1] = v
            quat_hist[k + 1] = q

        return pos_hist, vel_hist, quat_hist
