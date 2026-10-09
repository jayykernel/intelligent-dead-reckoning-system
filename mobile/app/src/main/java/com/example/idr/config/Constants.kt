package com.example.idr.config

/**
 * Centralized configuration constants for the Intelligent Dead Reckoning system on Android.
 * Physical constants, EKF parameters, gating thresholds, and sensor limits.
 *
 * Note: This mirrors the Python constants in [engine/config/constants.py] for shared parameters.
 * Platform-specific settings (e.g., loop frequencies) are defined separately in the respective
 * platform configurations to avoid unintended coupling.
 */
object Constants {
    // ======================
    // PHYSICAL CONSTANTS
    // ======================
    const val GRAVITY_MS2: Double = 9.80665  // Standard gravity, m/s^2
    const val EARTH_RADIUS_M: Double = 6378137.0  // WGS84 semi-major axis, meters

    // ======================
    // EKF DEFAULT PARAMETERS
    // ======================
    // Process noise standard deviations (continuous-time)
    const val SIGMA_ACC: Double = 0.2          // Accelerometer noise, m/s^2 / sqrt(Hz)
    const val SIGMA_GYRO: Double = 0.02        // Gyroscope noise, rad/s / sqrt(Hz)
    const val SIGMA_ACC_BIAS: Double = 0.001   // Accelerometer bias random walk, m/s^2 / sqrt(s)
    const val SIGMA_GYRO_BIAS: Double = 0.0001 // Gyroscope bias random walk, rad/s / sqrt(s)

    // Initial state uncertainty (diagonal of initial covariance P0)
    // These are 1-sigma values for the error state.
    const val INIT_POS_STD_M: Double = 10.0    // Initial position uncertainty, meters
    const val INIT_VEL_STD_MS: Double = 5.0    // Initial velocity uncertainty, m/s
    const val INIT_ATT_STD_DEG: Double = 10.0  // Initial attitude uncertainty, degrees (converted to rad in EKF)
    const val INIT_ACC_BIAS_STD_MS2: Double = 0.1  // Initial accelerometer bias uncertainty, m/s^2
    const val INIT_GYRO_BIAS_STD_RADS: Double = 0.01  // Initial gyroscope bias uncertainty, rad/s

    // ======================
    // GATING AND THRESHOLDS
    // ======================
    // Chi-squared gating significance level (probability of false alarm)
    const val ALPHA_NIS: Double = 0.01  // 1% false alarm probability

    // Degrees of freedom for each measurement type (used to look up chi2 threshold)
    // These are set per update type in the EKF, but we define them here for clarity and potential use in tests.
    // Note: The EKF computes the threshold based on the measurement dimension and ALPHA_NIS.
    const val MEAS_DIM_GNSS_POS: Int = 3
    const val MEAS_DIM_GNSS_VEL: Int = 3
    const val MEAS_DIM_HEADING: Int = 1
    const val MEAS_DIM_MAP_CROSS_TRACK: Int = 1
    const val MEAS_DIM_MAP_POS: Int = 3
    const val MEAS_DIM_ZUPT: Int = 3
    const val MEAS_DIM_ZARU: Int = 3
    const val MEAS_DIM_NHC: Int = 2

    // Maximum allowed NIS ratio (for logging or adaptive scaling, if used)
    // Not currently used in the EKF but kept for reference.
    const val MAX_NIS_RATIO: Double = 10.0

    // ======================
    // SENSOR LIMITS AND CLAMPS
    // ======================
    // Accelerometer sanity check (m/s^2)
    const val MAX_ACC_MAGNITUDE: Double = 30.0  // ~3g, reasonable for dynamics plus gravity
    // Gyroscope sanity check (rad/s)
    const val MAX_GYRO_MAGNITUDE: Double = 10.0  // ~570 deg/s, generous for consumer IMU
    // Magnetometer sanity check (microtesla)
    const val MAX_MAG_MAGNITUDE: Double = 100.0  // Earth's field ~50 uT, plus some tolerance

    // ======================
    // MODE TRANSITION TIMEOUTS (seconds)
    // ======================
    // Time to wait for GNSS reacquisition before declaring long outage
    const val GNSS_OUTAGE_SHORT_THRESHOLD_S: Double = 10.0   // Short outage: <10s
    const val GNSS_OUTAGE_LONG_THRESHOLD_S: Double = 60.0    // Long outage: >=60s (as per benchmark)

    // Dwell time for mode transition hysteresis (to prevent chattering)
    const val MODE_TRANSITION_DWELL_S: Double = 1.0

    // ======================
    // OUTAGE PREDICTION SCALING
    // ======================
    // Factors for adaptive covariance growth during GNSS outage
    const val OUTAGE_COVARIANCE_GROWTH_K: Double = 100.0   // Maximum scaling factor for position covariance
    const val OUTAGE_COVARIANCE_GROWTH_TAU: Double = 10.0  // Time constant for growth (seconds)

    // ======================
    // MISCELLANEOUS
    // ======================
    // Default IMU sample rate (Hz) used when not available from sensor
    const val DEFAULT_IMU_HZ: Double = 200.0
    // Default GNSS sample rate (Hz)
    const val DEFAULT_GNSS_HZ: Double = 5.0

    // Note: Platform-specific rates (e.g., Android UI loop, Edge engine loop)
    // are defined in the respective platform configs.
    // For Android, the fusion loop runs on a HandlerThread at 10 Hz (as per Phase 5) but the EKF dt is 0.1s (10 Hz).
    // However, note that the EKF dt is set to 0.1 in the ErrorStateEKF constructor, which matches the DEFAULT_IMU_HZ?
    // Actually, the IMU data comes at a higher rate, but we are integrating at 10 Hz?
    // We'll leave a comment that the EKF dt is set separately and should be consistent with the fusion loop.

    // In the Android MainActivity, we set the fusion loop to 10 Hz (100 ms).
    // We can define an Android-specific constant for the fusion loop period if needed, but note that the EKF dt is 0.1.
    // We'll not duplicate the fusion loop period here because it is already in MainActivity.
    // Instead, we note that the EKF dt (0.1) is the same as the fusion loop period (100 ms) in the Android implementation.
}