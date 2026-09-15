import argparse
import csv
import datetime
import math
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.sensors.data_types import (
    ImuSample, MagSample, GnssFix
)
from core.sensors.replay import SensorLogger

def convert_s1_dataset(s_csv_path: Path, v_csv_path: Path, out_phone_bin: Path, out_ref_csv: Path):
    print(f"Reading S-S1: {s_csv_path}")
    print(f"Reading V-S1: {v_csv_path}")

    out_phone_bin.parent.mkdir(parents=True, exist_ok=True)
    out_ref_csv.parent.mkdir(parents=True, exist_ok=True)

    # 1. Output formats
    logger = SensorLogger(out_phone_bin, format="bin")

    # Vehicle Reference CSV structure
    ref_columns = [
        "timestamp_ns", "latitude_deg", "longitude_deg", "altitude_m", "velocity_kmh",
        "heading_deg", "yaw_rate_deg_s", "wheel_speed_fl_rad_s", "wheel_speed_fr_rad_s",
        "wheel_speed_rl_rad_s", "wheel_speed_rr_rad_s", "steer_angle_deg", "accel_long_g", "accel_lat_g"
    ]

    ref_f = open(out_ref_csv, 'w', newline='', encoding='utf-8')
    ref_writer = csv.DictWriter(ref_f, fieldnames=ref_columns)
    ref_writer.writeheader()

    # State tracking
    last_gnss_lat = None
    last_gnss_lon = None
    input_rows = 0
    imu_count = 0
    mag_count = 0
    gnss_count = 0
    malformed_count = 0
    gnss_deduplicated = 0
    start_time_ns = 0

    with open(s_csv_path, 'r', encoding='utf-8', errors='replace') as sf, \
         open(v_csv_path, 'r', encoding='utf-8', errors='replace') as vf, \
         logger:

        s_reader = csv.reader(sf)
        v_reader = csv.reader(vf)

        s_header = [h.encode('ascii', 'ignore').decode('ascii').strip() for h in next(s_reader)]
        v_header = [h.strip() for h in next(v_reader)]

        def gidx(cp):
            for i, h in enumerate(s_header):
                if h.startswith(cp): return i
            raise ValueError(f"Missing S: {cp}")
        def vidx(cp):
            for i, h in enumerate(v_header):
                if h.startswith(cp): return i
            raise ValueError(f"Missing V: {cp}")

        si_lat = gidx('GPS LAT')
        si_lon = gidx('GPS LON')
        si_alt = gidx('GPS ALT')
        si_spd = gidx('GPS SPEED')
        si_acc = gidx('GPS ACC')
        si_ori = gidx('GPS ORIENTATION')
        si_sat = gidx('GPS SAT')
        si_time = gidx('TIME SINCE')
        si_date = gidx('DATE')
        si_acc_x = gidx('ACCELEROMETER X')
        si_acc_y = gidx('ACCELEROMETER Y')
        si_acc_z = gidx('ACCELEROMETER Z')
        si_gyr_yaw = gidx('GYROSCOPE Yaw')
        si_gyr_pit = gidx('GYROSCOPE Pitch')
        si_gyr_rol = gidx('GYROSCOPE Roll')
        si_mag_x = gidx('MAGNETIC FIELD X')
        si_mag_y = gidx('MAGNETIC FIELD Y')
        si_mag_z = gidx('MAGNETIC FIELD Z')

        vi_lat = vidx('Lat')
        vi_lon = vidx('Lon')
        vi_alt = vidx('Height')
        vi_vel = vidx('Vel')
        vi_hdg = vidx('Head')
        vi_yaw = vidx('Yaw Rate')
        vi_wfl = vidx('Wheel Speed Front Left')
        vi_wfr = vidx('Wheel Speed Front Right')
        vi_wrl = vidx('Wheel Speed Rear Left')
        vi_wrr = vidx('Wheel Speed Rear Right')
        vi_str = vidx('Steer')
        vi_alg = vidx('Indicated Longitudinal Acceleration')
        vi_ala = vidx('Indicated Lateral Acceleration')

        t_start = None
        t_end = None

        for s_row, v_row in zip(s_reader, v_reader):
            input_rows += 1
            if len(s_row) < len(s_header) or len(v_row) < len(v_header):
                malformed_count += 1
                continue

            try:
                dt = datetime.datetime.strptime(s_row[si_date].strip(), "%Y-%m-%d %H:%M:%S:%f")
                row_ts_ns = int(dt.timestamp() * 1e9)
            except ValueError:
                rel_ms = float(s_row[si_time])
                if start_time_ns == 0:
                    start_time_ns = 1567917469000000000
                row_ts_ns = start_time_ns + int(rel_ms * 1e6)

            if start_time_ns == 0:
                start_time_ns = row_ts_ns
            if t_start is None:
                t_start = row_ts_ns
            t_end = row_ts_ns

            try:
                ax = float(s_row[si_acc_x])
                ay = float(s_row[si_acc_y])
                az = float(s_row[si_acc_z])
                gx = float(s_row[si_gyr_rol])
                gy = float(s_row[si_gyr_pit])
                gz = float(s_row[si_gyr_yaw])
                logger.log_imu(ImuSample(row_ts_ns, (ax, ay, az), (gx, gy, gz)))
                imu_count += 1
            except ValueError:
                malformed_count += 1
                continue

            try:
                mx = float(s_row[si_mag_x])
                my = float(s_row[si_mag_y])
                mz = float(s_row[si_mag_z])
                logger.log_mag(MagSample(row_ts_ns, (mx, my, mz)))
                mag_count += 1
            except ValueError:
                pass

            lat = s_row[si_lat]
            lon = s_row[si_lon]
            # GNSS Deduplication
            if (lat != last_gnss_lat) or (lon != last_gnss_lon):
                try:
                    f_spd = float(s_row[si_spd])
                    f_acc = float(s_row[si_acc])
                    sat_str = s_row[si_sat].split('/')[0].strip()
                    sat_count = int(sat_str) if sat_str else 0
                    spd_mps = max(0.0, f_spd / 3.6)
                    hdg_rad = math.radians(float(s_row[si_ori]))
                    gnss = GnssFix(
                        timestamp_ns=row_ts_ns,
                        latitude_deg=float(lat),
                        longitude_deg=float(lon),
                        altitude_m=float(s_row[si_alt]),
                        velocity_ned_mps=(spd_mps * math.cos(hdg_rad), spd_mps * math.sin(hdg_rad), 0.0),
                        horizontal_accuracy_m=f_acc,
                        vertical_accuracy_m=f_acc*2.0, # assumed
                        speed_accuracy_mps=1.0,        # assumed
                        satellite_count=sat_count
                    )
                    logger.log_gnss(gnss)
                    gnss_count += 1
                    last_gnss_lat = lat
                    last_gnss_lon = lon
                except (ValueError, ZeroDivisionError):
                    pass
            else:
                gnss_deduplicated += 1

            try:
                ref_writer.writerow({
                    "timestamp_ns": row_ts_ns,
                    "latitude_deg": float(v_row[vi_lat]),
                    "longitude_deg": float(v_row[vi_lon]),
                    "altitude_m": float(v_row[vi_alt]),
                    "velocity_kmh": float(v_row[vi_vel]),
                    "heading_deg": float(v_row[vi_hdg]),
                    "yaw_rate_deg_s": float(v_row[vi_yaw]),
                    "wheel_speed_fl_rad_s": float(v_row[vi_wfl]),
                    "wheel_speed_fr_rad_s": float(v_row[vi_wfr]),
                    "wheel_speed_rl_rad_s": float(v_row[vi_wrl]),
                    "wheel_speed_rr_rad_s": float(v_row[vi_wrr]),
                    "steer_angle_deg": float(v_row[vi_str]),
                    "accel_long_g": float(v_row[vi_alg]),
                    "accel_lat_g": float(v_row[vi_ala]),
                })
            except ValueError:
                pass

    ref_f.close()

    dur = (t_end - t_start) / 1e9 if t_start and t_end else 0
    eff_i = imu_count / dur if dur > 0 else 0
    eff_g = gnss_count / dur if dur > 0 else 0

    print("\n--- Adapter Processing Complete ---")
    print(f"Total Rows Parsed: {input_rows}")
    print(f"Duration: {dur:.1f} seconds")
    print(f"Output IMU Events: {imu_count} (Effective Rate: {eff_i:.1f} Hz)")
    print(f"Output MAG Events: {mag_count} (Effective Rate: {eff_i:.1f} Hz)")
    print(f"Output GNSS Events: {gnss_count} (Effective Rate: {eff_g:.1f} Hz)")
    print(f"Deduplicated GNSS Samples: {gnss_deduplicated}")
    print(f"Reference Samples: {input_rows - malformed_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--s1-csv", default="data/raw/io-vnbd/S1/S-S1.csv")
    parser.add_argument("--v1-csv", default="data/raw/io-vnbd/S1/V-S1.csv")
    parser.add_argument("--out-bin", default="data/processed/io_vnbd_s1_phone.bin")
    parser.add_argument("--out-ref", default="data/processed/io_vnbd_s1_reference.csv")
    args = parser.parse_args()

    convert_s1_dataset(Path(args.s1_csv), Path(args.v1_csv), Path(args.out_bin), Path(args.out_ref))
