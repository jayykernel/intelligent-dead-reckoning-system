import sys, os
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from training.data_loader import load_iovnbd_session, preprocess_session
from engine.calibration.calibrator import CalibrationEngine
import itertools

def test_discovery(driver, session):
    try:
        s_df, v_df = load_iovnbd_session("data/raw", driver, session)
        synced = preprocess_session(s_df, v_df, target_dt=0.1)
    except Exception as e:
        print(f"Skipping {session}: {e}")
        return

    acc = synced[['acc_x', 'acc_y', 'acc_z']].values
    gyro = synced[['gyro_x', 'gyro_y', 'gyro_z']].values
    speed = synced['gt_speed'].values
    heading = synced['gt_heading'].values
    dt = 0.1

    calib = CalibrationEngine()
    # 1. Calibrate frame with raw IMU
    calib.calibrate_from_session(acc[:1200], gyro[:1200], speed[:1200], dt=dt)

    moving = speed > 2.0
    h_diff = (np.diff(heading) + 180) % 360 - 180
    h_rate = np.radians(h_diff) / dt
    h_rate = np.concatenate(([0], h_rate))
    h_rate_s = np.convolve(h_rate, np.ones(5)/5, mode='same')
    yaw_rate_veh_gt = -h_rate_s # vehicle yaw rate is opposite to clockwise heading change

    best_corr = -999.0
    best_perm = (0, 1, 2)
    best_signs = (1, 1, 1)

    for perm in itertools.permutations([0, 1, 2]):
        for s1 in [1, -1]:
            for s2 in [1, -1]:
                for s3 in [1, -1]:
                    calib.set_gyro_alignment(perm, (s1, s2, s3))
                    _, gyro_veh = calib.apply(acc, gyro)
                    c = np.corrcoef(gyro_veh[moving, 2], yaw_rate_veh_gt[moving])[0, 1]
                    if c > best_corr:
                        best_corr = c
                        best_perm = perm
                        best_signs = (s1, s2, s3)

    calib.set_gyro_alignment(best_perm, best_signs)
    print(f"[{session}] Best Corr: {best_corr:+.4f} | Perm: {best_perm}, Signs: {best_signs}")

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
    test_discovery(driver, s)
