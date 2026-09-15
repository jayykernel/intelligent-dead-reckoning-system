import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.sensors.replay import SensorReplayIterator, ImuSample, MagSample, GnssFix, ReplayConfig
import math

def test_smoke():
    log_path = Path("data/processed/io_vnbd_s1_phone.bin")
    iterator = SensorReplayIterator(log_path, config=ReplayConfig(playback_speed=99999999.0))
    
    imu_count = 0
    test_complete = False
    
    for sample in iterator:
        if isinstance(sample, ImuSample) and imu_count == 0:
            print("--- FIRST IMU SAMPLE ---")
            print(f"Time (ns): {sample.timestamp_ns}")
            print(f"Accel(XYZ m/s2): {sample.accel_m_s2}")
            print(f"Gyro (XYZ rad/s): {sample.gyro_rad_s}")
            imu_count += 1
            
        elif isinstance(sample, MagSample) and imu_count == 1:
            print("--- FIRST MAG SAMPLE ---")
            print(f"Time (ns): {sample.timestamp_ns}")
            print(f"Mag(XYZ uT): {sample.magnetic_field_ut}")
            imu_count += 1
            
        elif isinstance(sample, GnssFix) and not test_complete:
            print("--- FIRST GNSS FIX ---")
            print(f"Time (ns): {sample.timestamp_ns}")
            print(f"Lat/Lon/Alt: {sample.latitude_deg}, {sample.longitude_deg}, {sample.altitude_m}")
            print(f"Velocity NED (m/s): {sample.velocity_ned_mps}")
            print(f"Accuracy H/V/S: {sample.horizontal_accuracy_m}, {sample.vertical_accuracy_m}, {sample.speed_accuracy_mps}")
            print(f"Satellites: {sample.satellite_count}")
            test_complete = True
            break

if __name__ == "__main__":
    test_smoke()
