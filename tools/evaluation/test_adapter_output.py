import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.sensors.replay import SensorReplayIterator, ImuSample, MagSample, GnssFix, ReplayConfig

def test_read():
    log_path = Path("data/processed/io_vnbd_s1_phone.bin")
    if not log_path.exists():
        print("Log path missing!")
        sys.exit(1)
        
    config = ReplayConfig(playback_speed=99999999.0) # Maximum speed to bypass sleep
    iterator = SensorReplayIterator(log_path, config=config)
    
    imu = 0
    gnss = 0
    mag = 0
    bad = 0
    for sample in iterator:
        if isinstance(sample, ImuSample):
            imu += 1
        elif isinstance(sample, GnssFix):
            gnss += 1
            if gnss == 1:
                print(f"First GNSS: Lat={sample.latitude_deg}, Alt={sample.altitude_m}, Sat={sample.satellite_count}")
        elif isinstance(sample, MagSample):
            mag += 1
        else:
            bad += 1
            
    print(f"Verified IMU: {imu}, GNSS: {gnss}, MAG: {mag}, BAD: {bad}")
    assert imu == 51746
    assert mag == 51746
    assert gnss == 532
    assert bad == 0
    
if __name__ == "__main__":
    test_read()
