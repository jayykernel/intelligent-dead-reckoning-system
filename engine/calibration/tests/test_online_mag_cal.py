import unittest
import numpy as np
from engine.calibration.online_mag_cal import OnlineMagnetometerCalibrator


class TestOnlineMagnetometerCalibrator(unittest.TestCase):
    def setUp(self):
        self.calibrator = OnlineMagnetometerCalibrator(
            window_size=500,
            min_samples=50,
            field_strength=48.0,
            update_interval=5
        )

    def test_insufficient_excitation(self):
        # Generate data with no angular variation (almost constant)
        const_mag = np.array([10.0, 20.0, 30.0])
        for _ in range(100):
            noise = np.random.normal(0, 0.1, 3)
            self.calibrator.update(const_mag + noise)

        status = self.calibrator.get_calibration_status()
        self.assertFalse(status['is_calibrated'])
        self.assertFalse(self.calibrator.is_calibrated)

    def test_2d_planar_fit_fallback(self):
        # Generate 2D circle in XY plane with hard iron offset
        true_offset = np.array([15.0, -25.0, 5.0])
        angles = np.linspace(0, 4 * np.pi, 200)
        radius = 45.0

        for ang in angles:
            # Z is essentially constant -> degenerate 3D ellipsoid, will trigger 2D planar fallback
            sample = np.array([
                true_offset[0] + radius * np.cos(ang) + np.random.normal(0, 0.2),
                true_offset[1] + radius * np.sin(ang) + np.random.normal(0, 0.2),
                true_offset[2] + np.random.normal(0, 0.1)
            ])
            self.calibrator.update(sample)

        status = self.calibrator.get_calibration_status()
        self.assertTrue(status['is_calibrated'])
        np.testing.assert_allclose(self.calibrator.mag_hard_iron[:2], true_offset[:2], atol=2.0)
        self.assertGreater(self.calibrator.calibration_quality, 0.5)

    def test_3d_ellipsoid_fit(self):
        # Generate full 3D sphere/ellipsoid with known hard iron offset
        true_offset = np.array([12.0, -18.0, 25.0])
        r = 48.0

        np.random.seed(42)
        n_samples = 300
        u = np.random.uniform(0, 1, n_samples)
        v = np.random.uniform(0, 1, n_samples)
        theta = np.arccos(2 * u - 1)
        phi = 2 * np.pi * v

        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)

        for i in range(n_samples):
            sample = np.array([x[i], y[i], z[i]]) + true_offset + np.random.normal(0, 0.3, 3)
            self.calibrator.update(sample)

        status = self.calibrator.get_calibration_status()
        self.assertTrue(status['is_calibrated'])
        np.testing.assert_allclose(self.calibrator.mag_hard_iron, true_offset, atol=3.0)
        self.assertGreater(self.calibrator.calibration_quality, 0.7)

        # Test applying calibration
        test_pt = np.array([r, 0, 0]) + true_offset
        calibrated_pt = self.calibrator.apply_calibration(test_pt)
        self.assertAlmostEqual(np.linalg.norm(calibrated_pt), r, delta=3.0)

    def test_outlier_filtering(self):
        # Generate circle with some severe outliers
        true_offset = np.array([10.0, 10.0, 0.0])
        angles = np.linspace(0, 2 * np.pi, 100)
        radius = 45.0

        for i, ang in enumerate(angles):
            if i % 10 == 0:
                # Glitch / spike
                sample = np.array([500.0, -500.0, 1000.0])
            else:
                sample = np.array([
                    true_offset[0] + radius * np.cos(ang),
                    true_offset[1] + radius * np.sin(ang),
                    0.0
                ])
            self.calibrator.update(sample)

        status = self.calibrator.get_calibration_status()
        self.assertTrue(status['is_calibrated'])
        np.testing.assert_allclose(self.calibrator.mag_hard_iron[:2], true_offset[:2], atol=3.0)

    def test_reset(self):
        self.calibrator.is_calibrated = True
        self.calibrator.mag_hard_iron = np.array([1.0, 2.0, 3.0])
        self.calibrator.reset()
        self.assertFalse(self.calibrator.is_calibrated)
        np.testing.assert_array_equal(self.calibrator.mag_hard_iron, np.zeros(3))
        self.assertEqual(len(self.calibrator.mag_buffer), 0)


if __name__ == '__main__':
    unittest.main()
