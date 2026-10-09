import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from engine.config.constants import (
    GRAVITY_MS2,
    EARTH_RADIUS_M,
    SIGMA_ACC,
    SIGMA_GYRO,
    SIGMA_ACC_BIAS,
    SIGMA_GYRO_BIAS,
    INIT_POS_STD_M,
    INIT_VEL_STD_MS,
    INIT_ATT_STD_DEG,
    ALPHA_NIS,
    MAX_ACC_MAGNITUDE,
    MAX_GYRO_MAGNITUDE,
    GNSS_OUTAGE_LONG_THRESHOLD_S,
    GNSS_OUTAGE_SHORT_THRESHOLD_S,
    DEFAULT_IMU_HZ,
    DEFAULT_GNSS_HZ,
)

class TestConfig(unittest.TestCase):
    def test_physical_constants(self):
        self.assertAlmostEqual(GRAVITY_MS2, 9.80665)
        self.assertAlmostEqual(EARTH_RADIUS_M, 6378137.0)

    def test_ekf_process_noise_and_uncertainty(self):
        self.assertGreater(SIGMA_ACC, 0.0)
        self.assertGreater(SIGMA_GYRO, 0.0)
        self.assertGreater(SIGMA_ACC_BIAS, 0.0)
        self.assertGreater(SIGMA_GYRO_BIAS, 0.0)
        self.assertGreater(INIT_POS_STD_M, 0.0)
        self.assertGreater(INIT_VEL_STD_MS, 0.0)
        self.assertGreater(INIT_ATT_STD_DEG, 0.0)

    def test_gating_and_sensor_limits(self):
        self.assertAlmostEqual(ALPHA_NIS, 0.01)
        self.assertGreater(MAX_ACC_MAGNITUDE, GRAVITY_MS2)
        self.assertGreater(MAX_GYRO_MAGNITUDE, 0.0)

    def test_timeouts_and_frequencies(self):
        self.assertEqual(GNSS_OUTAGE_LONG_THRESHOLD_S, 60.0)
        self.assertEqual(GNSS_OUTAGE_SHORT_THRESHOLD_S, 10.0)
        self.assertGreater(DEFAULT_IMU_HZ, 0.0)
        self.assertGreater(DEFAULT_GNSS_HZ, 0.0)

if __name__ == '__main__':
    unittest.main()


