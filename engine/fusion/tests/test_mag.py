import sys
sys.path.insert(0, r"C:\dev\dead reckoning proto")
from engine.calibration.calibrator import CalibrationEngine
import numpy as np

calib = CalibrationEngine()
# Generate degenerate data
data = np.random.randn(100, 3) * 0.1 # Very small spread
data[:, 2] = 0.0 # completely planar
calib.calibrate_magnetometer(data)

print("Is calibrated?", calib.mag_is_calibrated)
print("Quality?", calib.mag_calibration_quality)
if calib.mag_is_calibrated:
    print("Matrix:", calib.mag_soft_iron)
