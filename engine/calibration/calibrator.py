import numpy as np
from typing import Dict, Tuple, Optional

class CalibrationEngine:
    """
    Estimates Phone-to-Vehicle frame alignment and IMU biases.

    Vehicle frame convention (Right-Handed):
    X: Right
    Y: Forward
    Z: Up

    Returns:
    - R_phone_to_veh: 3x3 rotation matrix
    - gyro_bias: (3,) rad/s
    - accel_bias: (3,) m/s^2

    Also supports ellipsoid-fit magnetometer calibration (N8) for absolute heading
    during GNSS outages.
    """

    def __init__(self):
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.R_phone_to_veh = np.eye(3)
        self.gyro_permutation = (0, 1, 2)  # (x, y, z) default
        self.gyro_signs = (1, 1, 1)        # (x, y, z) signs

        self.is_calibrated = False
        self.alignment_score = 0.0

        # Magnetometer calibration state (N8)
        self.mag_hard_iron = np.zeros(3)      # Hard-iron offset (center of ellipsoid)
        self.mag_soft_iron = np.eye(3)         # Soft-iron correction matrix
        self.mag_is_calibrated = False
        self.mag_calibration_quality = 0.0     # 0.0 to 1.0

    def set_gyro_alignment(self, permutation: Tuple[int, int, int], signs: Tuple[int, int, int]):
        """Set dynamic gyro axis permutation and sign inversion."""
        self.gyro_permutation = permutation
        self.gyro_signs = signs

    def calibrate_from_session(self, acc: np.ndarray, gyro: np.ndarray, speed: np.ndarray, dt: float = 0.1) -> bool:
        """
        Run calibration on a segment of data (e.g., first 120 seconds).
        acc: (N, 3)
        gyro: (N, 3)
        speed: (N,) true or GPS speed

        Returns True if successful, False if insufficient dynamic data.
        """
        N = len(acc)

        # 1. Isolate stationary periods for Gyro Bias and Gravity Down
        stationary_mask = speed < 0.2
        if np.sum(stationary_mask) < 20:
            stationary_mask = speed < 0.5

        if np.sum(stationary_mask) >= 10:
            stat_acc = acc[stationary_mask]
            stat_gyro = gyro[stationary_mask]
            self.gyro_bias = np.mean(stat_gyro, axis=0)
            g_phone = np.mean(stat_acc, axis=0)

            # Check if stationary gravity indicates a heavy tilt (e.g. motorbike on side stand)
            # If so, and we have clean steady upright driving data, prefer upright driving gravity
            gyro_norm = np.linalg.norm(gyro, axis=1)
            upright_mask = (speed > 1.0) & (gyro_norm < 0.2)
            if np.sum(upright_mask) > 20:
                g_upright = np.mean(acc[upright_mask], axis=0)
                # If angle between stat gravity and upright gravity > 20 deg, use upright gravity
                cos_ang = np.dot(g_phone, g_upright) / (np.linalg.norm(g_phone) * np.linalg.norm(g_upright))
                if cos_ang < np.cos(np.radians(15.0)):
                    g_phone = g_upright
        else:
            # Fallback to upright driving
            gyro_norm = np.linalg.norm(gyro, axis=1)
            upright_mask = (speed > 1.0) & (gyro_norm < 0.2)
            if np.sum(upright_mask) > 20:
                g_phone = np.mean(acc[upright_mask], axis=0)
                self.gyro_bias = np.zeros(3)
            else:
                print("  WARNING: Not enough stationary data for accurate gyro bias / gravity alignment.")
                return False

        # Vehicle Z (Up) points opposite to gravity
        z_v_phone = g_phone / np.linalg.norm(g_phone)

        # 2. Isolate forward acceleration periods for Forward Axis
        ds = np.gradient(speed, dt)

        # Look for strong forward acceleration
        accel_mask = ds > 0.5  # accelerating at > 0.5 m/s^2
        if np.sum(accel_mask) < 10:
            # Relax for gentler accelerations (two-wheelers in traffic)
            accel_mask = ds > 0.2
            if np.sum(accel_mask) < 10:
                print("  WARNING: Not enough strong forward acceleration to identify vehicle Y-axis.")
                return False

        fwd_acc = acc[accel_mask] - g_phone

        # Average the forward acceleration in phone frame
        a_fwd_phone = np.mean(fwd_acc, axis=0)

        # Project onto the horizontal plane (orthogonal to Z)
        y_v_phone = a_fwd_phone - np.dot(a_fwd_phone, z_v_phone) * z_v_phone
        norm_y = np.linalg.norm(y_v_phone)
        if norm_y < 0.05:
            print("  WARNING: Forward acceleration is co-linear with gravity or too weak. Cannot align.")
            return False

        y_v_phone = y_v_phone / norm_y

        # 3. Compute X (Right)
        x_v_phone = np.cross(y_v_phone, z_v_phone)

        # 4. Construct Rotation Matrix
        R_veh_to_phone = np.column_stack([x_v_phone, y_v_phone, z_v_phone])
        self.R_phone_to_veh = R_veh_to_phone.T

        # Score is based on amount of data
        self.alignment_score = min(1.0, np.sum(accel_mask) / 50.0)
        self.is_calibrated = True

        # Accel bias in vehicle frame:
        # Since we perfectly align Vehicle Z with g_phone, X and Y read 0 when stationary.
        # However, Z reads norm(g_phone).
        # Standard gravity in our EKF is 9.80665, so the difference is a sensor scale/bias error.
        self.accel_bias = np.array([0.0, 0.0, np.linalg.norm(g_phone) - 9.80665], dtype=np.float64)

        return True

    def calibrate_magnetometer(self, mag_data: np.ndarray) -> bool:
        """
        Ellipsoid-fit magnetometer calibration (N8).

        Fits a 3D ellipsoid to raw magnetometer samples to estimate hard-iron offset
        and soft-iron distortion matrix. Requires diverse heading coverage (ideally
        a figure-8 calibration maneuver or sufficient driving with varied headings).

        mag_data: (N, 3) raw magnetometer measurements in phone/sensor frame (uT)

        Returns True if calibration was successful, False otherwise.
        """
        N = len(mag_data)
        if N < 50:
            print("  WARNING: Not enough magnetometer samples for calibration (need >= 50).")
            return False

        # Remove obvious outliers (measurements > 3 sigma from median)
        mag_norms = np.linalg.norm(mag_data, axis=1)
        median_norm = np.median(mag_norms)
        std_norm = np.std(mag_norms)
        inlier_mask = np.abs(mag_norms - median_norm) < 3.0 * std_norm
        mag_clean = mag_data[inlier_mask]

        if len(mag_clean) < 30:
            print("  WARNING: Too few inlier magnetometer samples after outlier removal.")
            return False

        # Least-squares ellipsoid fit
        # Model: (m - c)^T A (m - c) = 1
        # Linearised: a*x^2 + b*y^2 + c*z^2 + d*xy + e*xz + f*yz + g*x + h*y + i*z = 1
        x, y, z = mag_clean[:, 0], mag_clean[:, 1], mag_clean[:, 2]

        # Check spatial excitation: need reasonable spread across at least 2 axes
        spread_xyz = np.ptp(mag_clean, axis=0)
        if np.sum(spread_xyz > 15.0) < 2:
            print(f"  WARNING: Magnetometer data lacks angular excitation (spread={spread_xyz}). Calibration skipped.")
            return False

        D = np.column_stack([
            x**2, y**2, z**2,
            2*x*y, 2*x*z, 2*y*z,
            x, y, z
        ])

        ones = np.ones(len(mag_clean))

        try:
            # Solve D @ params = ones in least-squares sense
            params, residuals, rank, sv = np.linalg.lstsq(D, ones, rcond=None)
        except np.linalg.LinAlgError:
            print("  WARNING: Magnetometer ellipsoid fit failed (singular matrix).")
            return False

        # Extract ellipsoid parameters
        a, b, c, d, e, f, g, h, i = params

        # Construct the quadratic form matrix A
        A = np.array([
            [a, d, e],
            [d, b, f],
            [e, f, c]
        ])

        # Linear terms vector
        bv = np.array([g, h, i])

        # Check that A is positive definite (valid ellipsoid). If not, attempt 2D planar fallback.
        eigvals = np.linalg.eigvalsh(A)
        if np.any(eigvals <= 0):
            print("  WARNING: Magnetometer ellipsoid fit produced non-positive-definite matrix (degenerate planar data).")
            # Universal Fix 3: 2D Planar Calibration Fallback
            # Attempt 2D circle fit on horizontal (X-Y) plane assuming level vehicle motion
            print("  Attempting 2D planar circle fit fallback for horizontal hard-iron estimation...")

            # Project magnetometer data onto horizontal plane (X-Y)
            mag_xy = mag_clean[:, :2]  # (N, 2)

            # 2D circle fit: (x - cx)^2 + (y - cy)^2 = r^2
            # Least-squares: fit circle center (cx, cy)
            A_2d = np.column_stack([mag_xy[:, 0], mag_xy[:, 1], np.ones(len(mag_xy))])
            b_2d = mag_xy[:, 0]**2 + mag_xy[:, 1]**2

            try:
                sol_2d = np.linalg.lstsq(A_2d, b_2d, rcond=None)[0]
                cx = sol_2d[0] / 2.0
                cy = sol_2d[1] / 2.0

                # Set hard-iron offset (X, Y from circle fit, Z from mean)
                self.mag_hard_iron = np.array([cx, cy, np.mean(mag_clean[:, 2])])

                # Estimate 2D soft-iron correction (assume isotropic in horizontal plane)
                mag_corrected_xy = mag_xy - np.array([cx, cy])
                radii = np.linalg.norm(mag_corrected_xy, axis=1)
                avg_radius_2d = np.mean(radii)

                # Normalize to expected horizontal Earth field magnitude (~45 uT)
                scale_2d = 45.0 / avg_radius_2d if avg_radius_2d > 1.0 else 1.0
                self.mag_soft_iron = np.diag([scale_2d, scale_2d, 1.0])

                # Assess 2D calibration quality
                residuals_2d = np.abs(radii - avg_radius_2d)
                residual_std_2d = np.std(residuals_2d) / avg_radius_2d
                self.mag_calibration_quality = float(np.clip(1.0 - residual_std_2d / 0.15, 0.1, 0.85))

                self.mag_is_calibrated = True
                print(f"  2D planar magnetometer calibration: hard_iron=({self.mag_hard_iron[0]:.1f}, {self.mag_hard_iron[1]:.1f}, {self.mag_hard_iron[2]:.1f}) uT, quality={self.mag_calibration_quality:.2f}")

                return True
            except (np.linalg.LinAlgError, ValueError) as e:
                print(f"  WARNING: 2D planar fallback also failed: {e}")
                return False

        # Hard-iron offset: center = -0.5 * A^{-1} @ bv
        try:
            A_inv = np.linalg.inv(A)
        except np.linalg.LinAlgError:
            print("  WARNING: Cannot invert ellipsoid matrix.")
            return False

        self.mag_hard_iron = -0.5 * A_inv @ bv

        # Soft-iron correction: transform ellipsoid to sphere
        # A = R @ diag(eigenvalues) @ R^T
        # Correction matrix W = sqrt(diag(eigenvalues)) @ R^T (normalised to unit sphere)
        eigvals_A, eigvecs_A = np.linalg.eigh(A)
        sqrt_eigvals = np.sqrt(eigvals_A)

        # Normalise so the average radius matches expected Earth field (~48 uT)
        avg_radius = np.mean(1.0 / sqrt_eigvals)
        sqrt_eigvals_norm = sqrt_eigvals * avg_radius

        self.mag_soft_iron = np.diag(sqrt_eigvals_norm) @ eigvecs_A.T

        # Assess calibration quality based on:
        # 1. Coverage: spread of heading angles
        # 2. Residual error after correction
        corrected = (mag_clean - self.mag_hard_iron) @ self.mag_soft_iron.T
        corrected_norms = np.linalg.norm(corrected, axis=1)
        residual_std = np.std(corrected_norms) / np.mean(corrected_norms)

        # Quality: 1.0 if residual_std < 2%, 0.0 if > 20%
        self.mag_calibration_quality = float(np.clip(1.0 - (residual_std - 0.02) / 0.18, 0.0, 1.0))

        self.mag_is_calibrated = True
        print(f"  Magnetometer calibration: hard_iron=({self.mag_hard_iron[0]:.1f}, {self.mag_hard_iron[1]:.1f}, {self.mag_hard_iron[2]:.1f}) uT, quality={self.mag_calibration_quality:.2f}")

        return True

    def apply_mag_calibration(self, mag_raw: np.ndarray) -> np.ndarray:
        """
        Apply hard-iron and soft-iron correction to a raw magnetometer measurement.

        mag_raw: (3,) raw magnetometer vector
        Returns: (3,) calibrated magnetometer vector
        """
        if not self.mag_is_calibrated:
            return mag_raw

        return (mag_raw - self.mag_hard_iron) @ self.mag_soft_iron.T

    def apply(self, acc: np.ndarray, gyro: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Apply calibration and rotation to turn raw phone IMU into vehicle-frame IMU.
        """
        # 1. Rotate gyro to vehicle frame
        # Apply bias correction in the raw frame FIRST
        gyro_corrected = gyro - self.gyro_bias
        gyro_veh = gyro_corrected @ self.R_phone_to_veh.T

        # 2. Rotate raw acc to vehicle frame first
        acc_veh = acc @ self.R_phone_to_veh.T
        # Subtract accel_bias which is defined in Vehicle Frame
        acc_veh = acc_veh - self.accel_bias

        return acc_veh, gyro_veh

    def check_misalignment_trigger(self, acc_window: np.ndarray, speed_window: np.ndarray, dt: float = 0.1, threshold_deg: float = 10.0) -> bool:
        """
        Check if the phone mount has shifted by comparing new data against the existing calibration.
        Returns True if a recalibration should be triggered (misalignment > threshold_deg).
        """
        if not self.is_calibrated:
            return True

        N = len(acc_window)
        stationary_mask = speed_window < 0.5

        # 1. Check gravity alignment if stationary
        if np.sum(stationary_mask) >= 10:
            stat_acc = acc_window[stationary_mask]
            g_phone = np.mean(stat_acc, axis=0)
            z_v_phone = g_phone / np.linalg.norm(g_phone)

            # Existing Z vector from inverse rotation matrix (3rd row)
            old_z_v_phone = self.R_phone_to_veh[2, :]

            cos_err = np.dot(z_v_phone, old_z_v_phone)
            err_deg = np.degrees(np.arccos(np.clip(cos_err, -1.0, 1.0)))

            if err_deg > threshold_deg:
                return True

        # 2. Check forward alignment if accelerating
        ds = np.gradient(speed_window, dt)
        accel_mask = ds > 0.5
        if np.sum(accel_mask) >= 10:
            stat_acc = acc_window[stationary_mask] if np.sum(stationary_mask) > 0 else np.array([0,0,9.81]) @ self.R_phone_to_veh  # hack fallback
            if np.sum(stationary_mask) > 0:
                g_phone = np.mean(stat_acc, axis=0)

            fwd_acc = acc_window[accel_mask] - g_phone
            a_fwd_phone = np.mean(fwd_acc, axis=0)

            z_v_phone = g_phone / np.linalg.norm(g_phone)
            y_v_phone = a_fwd_phone - np.dot(a_fwd_phone, z_v_phone) * z_v_phone
            norm_y = np.linalg.norm(y_v_phone)

            if norm_y > 1e-3:
                y_v_phone = y_v_phone / norm_y
                old_y_v_phone = self.R_phone_to_veh[1, :]

                cos_err = np.dot(y_v_phone, old_y_v_phone)
                err_deg = np.degrees(np.arccos(np.clip(cos_err, -1.0, 1.0)))

                if err_deg > threshold_deg:
                    return True

        return False
