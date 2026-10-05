import sys, os
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine

def check_session(driver, session):
    try:
        s_df, v_df = load_iovnbd_session("data/raw", driver, session)
        synced = preprocess_session(s_df, v_df, target_dt=0.1)
    except Exception as e:
        return

    acc = synced[['acc_x', 'acc_y', 'acc_z']].values
    gyro = synced[['gyro_x', 'gyro_y', 'gyro_z']].values
    speed = synced['gt_speed'].values
    heading = synced['gt_heading'].values
    dt = 0.1

    calib = CalibrationEngine()
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)
    calib.set_gyro_alignment((0, 1, 2), (1, 1, 1))

    moving = speed > 2.0
    h_diff = (np.diff(heading) + 180) % 360 - 180
    h_rate = np.radians(h_diff) / dt
    h_rate = np.concatenate(([0], h_rate))
    h_rate_s = np.convolve(h_rate, np.ones(5)/5, mode='same')
    # Note: in ENU frame, heading is measured clockwise from North:
    # heading = atan2(E, N). When turning right (clockwise), heading increases, so d(heading)/dt > 0.
    # In vehicle frame (X=Right, Y=Forward, Z=Up), turning right corresponds to negative yaw rate (-Z).
    # Therefore, yaw_rate_veh = - d(heading)/dt.
    yaw_rate_veh_gt = -h_rate_s

    _, gyro_veh = calib.apply(acc, gyro)
    print(f"[{session}] Mean Gyro: {np.mean(gyro_veh, axis=0)}")
    print(f"[{session}] Std Gyro: {np.std(gyro_veh, axis=0)}")
    c = np.corrcoef(gyro_veh[moving, 2], yaw_rate_veh_gt[moving])[0, 1]
    print(f"[{session}] Identity Corr: {c:+.4f}")

sessions = [
    ("S (Driver A)", "S4"),
    ("V-Vf (Driver B)", "V-Vfa02"),
    ("Vta (Driver E)", "Vta26"),
    ("Vta (Driver E)", "Vta27"),
    ("Vta (Driver E)", "Vta28"),
    ("Vta (Driver E)", "Vta29"),
    ("Vta (Driver E)", "Vta30"),
    ("Vtb (Driver E)", "Vtb11"),
    ("Vtb (Driver E)", "Vtb12"),
    ("Vw (Driver D)", "Vw15"),
    ("Vw (Driver D)", "Vw16a"),
    ("Vw (Driver D)", "Vw16b"),
    ("Vw (Driver D)", "Vw17"),
]

for driver, s in sessions:
    check_session(driver, s)
