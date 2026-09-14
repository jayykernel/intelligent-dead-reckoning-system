import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

import numpy as np
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.filters.eskf import ErrorStateKalmanFilter
from core.sensors.data_types import ImuSample
from core.map.geometry import RoadSegment
from core.map.matcher import MapMatcher

def simulate_trajectory(use_map_matching: bool, bad_map: bool = False):
    # Initial state exactly on road moving North
    state = NavState(
        timestamp_ns=0, position_m=(0.0, 0.0, 0.0), velocity_mps=(10.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0), accel_bias_mps2=(0.0, 0.0, 0.0), gyro_bias_radps=(0.0, 0.0, 0.0)
    )
    ins = StrapdownINS(state, accel_noise_std=0.01, gyro_noise_std=0.001)
    eskf = ErrorStateKalmanFilter(ins)
    
    # Ground truth Road goes straight North
    good_segment = RoadSegment("r_good", (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0))
    # Bad Road goes North but incorrectly offset 40m East
    bad_segment = RoadSegment("r_bad", (0.0, 40.0, 0.0), (1000.0, 40.0, 0.0))
    
    segments = [bad_segment] if bad_map else [good_segment]
    matcher = MapMatcher(segments, cross_track_variance=1.0, along_track_variance=1e9)

    dt = 0.1
    time = 0
    num_steps = 100
    
    match_count = 0
    rej_count = 0
    drift_ct = 0.0
    
    for i in range(num_steps):
        time += dt
        ts_ns = int(time * 1e9)
        
        # Inject artificial cross-track acceleration error (pulling it East incorrectly)
        imu = ImuSample(ts_ns, (0.0, -0.5, -9.81), (0.0, 0.0, 0.0))
        ins.propagate(imu, propagate_covariance=True)
        
        # Accumulate theoretical drift
        drift_ct += 0.5 * (0.5) * (dt ** 2)
        
        if use_map_matching:
            res = matcher.match(eskf.ins.state)
            if res is not None:
                match_count += 1
                eskf.update_position(res.projected_pos_ned, res.observation_cov, gate=4.0)
            else:
                rej_count += 1
                
    pos = ins.state.position_m
    cov = ins.covariance[0:2, 0:2]
    
    return pos, cov, match_count, rej_count

def main():
    print("Phase 15: Map Matching Validation")
    print(f"{'Configuration':<25} | {'N Pos (Along)':<15} | {'E Pos (Cross)':<15} | {'Matches / Rej'}")
    print("-" * 75)
    
    # 1. No map matching (Pure DR)
    pos_dr, cov_dr, m_dr, r_dr = simulate_trajectory(False)
    print(f"{'No Map (Pure DR)':<25} | {pos_dr[0]:<15.2f} | {pos_dr[1]:<15.2f} | {m_dr} / {r_dr}")
    
    # 2. Good map matching
    pos_mm, cov_mm, m_mm, r_mm = simulate_trajectory(True, False)
    print(f"{'With Valid Map':<25} | {pos_mm[0]:<15.2f} | {pos_mm[1]:<15.2f} | {m_mm} / {r_mm}")
    
    # 3. Bad map (should be rejected)
    pos_bad, cov_bad, m_bad, r_bad = simulate_trajectory(True, True)
    print(f"{'With Distant/Bad Map':<25} | {pos_bad[0]:<15.2f} | {pos_bad[1]:<15.2f} | {m_bad} / {r_bad}")

if __name__ == "__main__":
    main()

    print("\nCovariances (N, E):")
    print(f"No Map: N={cov_dr[0,0]:.4f}, E={cov_dr[1,1]:.4f}")
    print(f"Match : N={cov_mm[0,0]:.4f}, E={cov_mm[1,1]:.4f}")
