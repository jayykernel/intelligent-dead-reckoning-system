"""
Online Magnetometer Calibration (N8 Experiment)

Implements recursive least-squares ellipsoid fitting for hard/soft iron
calibration during vehicle motion. Designed to run continuously during
GNSS-aided operation to compute session-specific offsets before GNSS outage.

This is an experiment for Phase 2 - Heading Observability.
"""

import numpy as np
from typing import Optional, Tuple
from collections import deque


class OnlineMagnetometerCalibrator:
    """
    Online magnetometer calibration using recursive least squares.
    Maintains running estimates of hard-iron offset and soft-iron matrix.
    """

    def __init__(
        self,
        window_size: int = 1000,
        min_samples: int = 50,
        forgetting_factor: float = 0.995,
        field_strength: float = 48.0,  # Expected Earth magnetic field strength (uT)
        update_interval: int = 10      # Update every N samples to reduce computation
    ):
        self.window_size = window_size
        self.min_samples = min_samples
        self.forgetting_factor = forgetting_factor
        self.field_strength = field_strength
        self.update_interval = update_interval

        # Data buffers
        self.mag_buffer = deque(maxlen=window_size)
        self.sample_count = 0

        # Calibration state
        self.is_calibrated = False
        self.mag_hard_iron = np.zeros(3)      # Hard-iron offset (uT)
        self.mag_soft_iron = np.eye(3)        # Soft-iron correction matrix
        self.calibration_quality = 0.0        # 0.0 to 1.0

        # RLS parameters for ellipsoid fitting
        # We'll use a simplified approach: collect batch and fit periodically
        self.last_update_sample = 0

        # Statistics for excitation detection
        self.field_samples = deque(maxlen=100)  # For field strength monitoring

    def update(self, mag_raw: np.ndarray) -> bool:
        """
        Update calibration with new magnetometer sample.

        Args:
            mag_raw: (3,) raw magnetometer measurement in sensor frame (uT)

        Returns:
            True if calibration was updated, False otherwise
        """
        if not np.all(np.isfinite(mag_raw)):
            return False

        self.mag_buffer.append(mag_raw.copy())
        self.sample_count += 1
        self.field_samples.append(np.linalg.norm(mag_raw))

        # Update periodically to balance computation and responsiveness
        if self.sample_count - self.last_update_sample >= self.update_interval:
            self.last_update_sample = self.sample_count
            return self._update_calibration()

        return False

    def _update_calibration(self) -> bool:
        """Perform ellipsoid fit on buffered data."""
        if len(self.mag_buffer) < self.min_samples:
            return False

        mag_data = np.array(self.mag_buffer)

        # Check for sufficient magnetic field excitation
        spread = np.ptp(mag_data, axis=0)  # Peak-to-peak in each axis
        if np.sum(spread > 15.0) < 2:  # Need excitation in at least 2 axes
            self.is_calibrated = False
            return False

        # Remove outliers using median absolute deviation
        mag_norms = np.linalg.norm(mag_data, axis=1)
        median_norm = np.median(mag_norms)
        mad = np.median(np.abs(mag_norms - median_norm))
        threshold = 3.0 * mad if mad > 0 else np.inf

        inlier_mask = np.abs(mag_norms - median_norm) < threshold
        mag_clean = mag_data[inlier_mask]

        if len(mag_clean) < self.min_samples:
            self.is_calibrated = False
            return False

        # Fit ellipsoid: (m - c)^T A (m - c) = 1
        # Linearized form for least squares
        x, y, z = mag_clean[:, 0], mag_clean[:, 1], mag_clean[:, 2]

        D = np.column_stack([
            x**2, y**2, z**2,
            2*x*y, 2*x*z, 2*y*z,
            x, y, z
        ])

        ones = np.ones(len(mag_clean))

        try:
            # Solve D @ params = ones
            params, residuals, rank, s = np.linalg.lstsq(D, ones, rcond=None)
        except np.linalg.LinAlgError:
            self.is_calibrated = False
            return False

        if len(params) != 9:
            self.is_calibrated = False
            return False

        # Extract ellipsoid parameters
        a, b, c, d, e, f, g, h, i = params

        # Construct quadratic form matrix A
        A = np.array([
            [a, d, e],
            [d, b, f],
            [e, f, c]
        ])

        # Check that A is positive definite
        eigvals = np.linalg.eigvalsh(A)
        if np.any(eigvals <= 0):
            # Degenerate case - try 2D planar fallback
            return self._fit_2d_planar(mag_clean)

        # Linear terms vector
        bv = np.array([g, h, i])

        # Hard-iron offset: center = -0.5 * A^{-1} @ bv
        try:
            A_inv = np.linalg.inv(A)
        except np.linalg.LinAlgError:
            self.is_calibrated = False
            return False

        self.mag_hard_iron = -0.5 * A_inv @ bv

        # Scale factor S from completing the square: (m - c)^T (A / S) (m - c) = 1
        # S = 1 + c^T A c = 1 - 0.5 * bv^T c
        S = 1.0 - 0.5 * float(np.dot(bv, self.mag_hard_iron))
        if S <= 0:
            return self._fit_2d_planar(mag_clean)

        # Soft-iron correction: transform ellipsoid (m - c)^T (A / S) (m - c) = 1
        # to a sphere of radius self.field_strength
        eigvals_A, eigvecs_A = np.linalg.eigh(A)
        if np.any(eigvals_A <= 0):
            return self._fit_2d_planar(mag_clean)

        # W = (field_strength / sqrt(S)) * V @ diag(sqrt(eigvals)) @ V^T
        sqrt_eigvals_norm = (self.field_strength / np.sqrt(S)) * np.sqrt(eigvals_A)
        self.mag_soft_iron = eigvecs_A @ np.diag(sqrt_eigvals_norm) @ eigvecs_A.T

        # Assess calibration quality
        corrected = (mag_clean - self.mag_hard_iron) @ self.mag_soft_iron.T
        corrected_norms = np.linalg.norm(corrected, axis=1)
        residual_std = np.std(corrected_norms) / np.mean(corrected_norms)

        # Quality: 1.0 if residual_std < 2%, 0.0 if > 20%
        self.calibration_quality = float(np.clip(1.0 - (residual_std - 0.02) / 0.18, 0.0, 1.0))

        self.is_calibrated = True
        return True

    def _fit_2d_planar(self, mag_data: np.ndarray) -> bool:
        """Fit 2D circle in horizontal plane assuming level motion."""
        # Use X-Y components (assume Z is vertical)
        mag_xy = mag_data[:, :2]

        # 2D circle fit: (x - cx)^2 + (y - cy)^2 = r^2
        # Least squares: fit circle center
        A_2d = np.column_stack([mag_xy[:, 0], mag_xy[:, 1], np.ones(len(mag_xy))])
        b_2d = mag_xy[:, 0]**2 + mag_xy[:, 1]**2

        try:
            sol_2d = np.linalg.lstsq(A_2d, b_2d, rcond=None)[0]
            cx = sol_2d[0] / 2.0
            cy = sol_2d[1] / 2.0
        except np.linalg.LinAlgError:
            self.is_calibrated = False
            return False

        # Set hard-iron (XY from circle fit, Z from mean)
        self.mag_hard_iron = np.array([cx, cy, np.mean(mag_data[:, 2])])

        # Estimate soft-iron correction
        mag_corrected_xy = mag_xy - np.array([cx, cy])
        radii = np.linalg.norm(mag_corrected_xy, axis=1)
        avg_radius = np.mean(radii)

        if avg_radius > 1.0:
            # Normalize to expected horizontal field (~45 uT at mid-latitudes)
            scale = 45.0 / avg_radius
            self.mag_soft_iron = np.diag([scale, scale, 1.0])
        else:
            self.mag_soft_iron = np.eye(3)

        # Assess 2D quality
        residuals_2d = np.abs(radii - avg_radius)
        residual_std_2d = np.std(residuals_2d) / avg_radius
        self.calibration_quality = float(np.clip(1.0 - residual_std_2d / 0.15, 0.1, 0.85))

        self.is_calibrated = True
        return True

    def apply_calibration(self, mag_raw: np.ndarray) -> np.ndarray:
        """
        Apply hard-iron and soft-iron correction to raw magnetometer measurement.

        Args:
            mag_raw: (3,) raw magnetometer measurement in sensor frame (uT)

        Returns:
            (3,) calibrated magnetometer measurement
        """
        if not self.is_calibrated:
            return mag_raw

        return (mag_raw - self.mag_hard_iron) @ self.mag_soft_iron.T

    def get_calibration_status(self) -> dict:
        """Get current calibration status for monitoring."""
        return {
            'is_calibrated': self.is_calibrated,
            'samples': self.sample_count,
            'buffer_size': len(self.mag_buffer),
            'hard_iron': self.mag_hard_iron.copy() if self.is_calibrated else None,
            'soft_iron': self.mag_soft_iron.copy() if self.is_calibrated else None,
            'quality': self.calibration_quality,
            'field_strength_estimate': np.mean(self.field_samples) if self.field_samples else 0.0
        }

    def reset(self):
        """Reset calibration state."""
        self.mag_buffer.clear()
        self.sample_count = 0
        self.last_update_sample = 0
        self.field_samples.clear()
        self.is_calibrated = False
        self.mag_hard_iron = np.zeros(3)
        self.mag_soft_iron = np.eye(3)
        self.calibration_quality = 0.0