"""
Centralized configuration constants for the Intelligent Dead Reckoning system.
Physical constants, EKF parameters, gating thresholds, and sensor limits.
"""

import numpy as np

# ======================
# PHYSICAL CONSTANTS
# ======================
GRAVITY_MS2 = 9.80665  # Standard gravity, m/s^2
EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis, meters
# Note: Additional geodetic constants are in the geodesy functions if needed elsewhere.

# ======================
# EKF DEFAULT PARAMETERS
# ======================
# Process noise standard deviations (continuous-time)
SIGMA_ACC = 0.2          # Accelerometer noise, m/s^2 / sqrt(Hz)
SIGMA_GYRO = 0.02        # Gyroscope noise, rad/s / sqrt(Hz)
SIGMA_ACC_BIAS = 0.001   # Accelerometer bias random walk, m/s^2 / sqrt(Hz) / sqrt(s)?? Actually units: m/s^2 / sqrt(s)
SIGMA_GYRO_BIAS = 0.0001 # Gyroscope bias random walk, rad/s / sqrt(s)

# Initial state uncertainty (diagonal of initial covariance P0)
# These are 1-sigma values for the error state.
INIT_POS_STD_M = 10.0    # Initial position uncertainty, meters
INIT_VEL_STD_MS = 5.0    # Initial velocity uncertainty, m/s
INIT_ATT_STD_DEG = 10.0  # Initial attitude uncertainty, degrees (converted to rad in EKF)
INIT_ACC_BIAS_STD_MS2 = 0.1  # Initial accelerometer bias uncertainty, m/s^2
INIT_GYRO_BIAS_STD_RADS = 0.01  # Initial gyroscope bias uncertainty, rad/s

# ======================
# GATING AND THRESHOLDS
# ======================
# Chi-squared gating significance level (probability of false alarm)
ALPHA_NIS = 0.01  # 1% false alarm probability

# Degrees of freedom for each measurement type (used to look up chi2 threshold)
# These are set per update type in the EKF, but we can define the common ones here.
# Note: The EKF computes the threshold based on the measurement dimension and ALPHA_NIS.
# We keep the dimension definitions here for clarity and potential use in tests.
MEAS_DIM_GNSS_POS = 3
MEAS_DIM_GNSS_VEL = 3
MEAS_DIM_HEADING = 1
MEAS_DIM_MAP_CROSS_TRACK = 1
MEAS_DIM_MAP_POS = 3
MEAS_DIM_ZUPT = 3
MEAS_DIM_ZARU = 3
MEAS_DIM_NHC = 2

# Maximum allowed NIS ratio (for logging or adaptive scaling, if used)
# Not currently used in the EKF but kept for reference.
MAX_NIS_RATIO = 10.0

# ======================
# SENSOR LIMITS AND CLAMPS
# ======================
# Accelerometer sanity check (m/s^2)
MAX_ACC_MAGNITUDE = 30.0  # ~3g, reasonable for dynamics plus gravity
# Gyroscope sanity check (rad/s)
MAX_GYRO_MAGNITUDE = 10.0  # ~570 deg/s, generous for consumer IMU
# Magnetometer sanity check (microtesla)
MAX_MAG_MAGNITUDE = 100.0  # Earth's field ~50 uT, plus some tolerance

# ======================
# MODE TRANSITION TIMEOUTS (seconds)
# ================== #
# Time to wait for GNSS reacquisition before declaring long outage
GNSS_OUTAGE_SHORT_THRESHOLD_S = 10.0   # Short outage: <10s
GNSS_OUTAGE_LONG_THRESHOLD_S = 60.0    # Long outage: >=60s (as per benchmark)

# Dwell time for mode transition hysteresis (to prevent chattering)
MODE_TRANSITION_DWELL_S = 1.0

# ======================
# OUTAGE PREDICTION SCALING
# ======================
# Factors for adaptive covariance growth during GNSS outage
OUTAGE_COVARIANCE_GROWTH_K = 100.0   # Maximum scaling factor for position covariance
OUTAGE_COVARIANCE_GROWTH_TAU = 10.0  # Time constant for growth (seconds)

# ======================
# MISCELLANEOUS
# ======================
# Default IMU sample rate (Hz) used when not available from sensor
DEFAULT_IMU_HZ = 200.0
# Default GNSS sample rate (Hz)
DEFAULT_GNSS_HZ = 5.0

# Note: Platform-specific rates (e.g., Android UI loop, Edge engine loop)
# are defined in the respective platform configs.