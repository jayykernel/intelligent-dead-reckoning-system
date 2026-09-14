import numpy as np
from core.map.geometry import RoadSegment
from core.map.matcher import MapMatcher
from core.navigation.state import NavState
from core.navigation.mechanization import StrapdownINS
from core.map.multi_hypothesis import MultiHypothesisTracker
from core.sensors.data_types import ImuSample

def run_flyover_simulation():
    # Trajectory goes along a road. Wait, an underpass and a flyover overlap horizontally.
    # Ground road (underpass) at z = 0.0
    underpass = RoadSegment("underpass", (0, 0, 0), (200, 0, 0), has_elevation=True)
    # Flyover at z = -15.0
    flyover = RoadSegment("flyover", (0, 0, -15), (200, 0, -15), has_elevation=True)
    
    # Create the MapMatcher and Tracker
    matcher = MapMatcher([underpass, flyover], max_distance_m=10.0, cross_track_variance=1.0, vertical_variance=4.0)
    tracker = MultiHypothesisTracker(matcher, max_hypotheses=3)
    
    # Initialize the vehicle going onto the flyover (it thinks it's near z=-12, maybe rising)
    initial_state = NavState(
        timestamp_ns=0,
        position_m=(0.0, 0.0, -2.0), # Slowly rising, ambiguous initially
        velocity_mps=(10.0, 0.0, 0.0),
        attitude_q_v2n=(1.0, 0.0, 0.0, 0.0),
        accel_bias_mps2=(0.0, 0.0, 0.0),
        gyro_bias_radps=(0.0, 0.0, 0.0)
    )
    ins = StrapdownINS(initial_state)
    ins.covariance[2, 2] = 25.0 # High vertical uncertainty initially (e.g. 5m std)
    
    tracker.initialize(ins)
    
    print("--- Time 0.0s: Approaching Intersection ---")
    tracker.update_map()
    for h in tracker.hypotheses:
        print(f"  {h.id}: {h.parent_segment_id} | Score: {h.score:.3f} | Z-pos: {h.ins.state.position_m[2]:.2f}m")
        
    print("\n--- Time 5.0s: Climbing the Ramp (Drifting Upwards) ---")
    # Simulate driving up the ramp for 5 seconds.
    for i in range(500):
        for h in tracker.hypotheses:
            h.ins.propagate(
                ImuSample(
                    timestamp_ns=int((i+1) * 1e7),
                    accel_m_s2=(0.0, 0.0, -9.80665 - 0.5), # Upward acceleration
                    gyro_rad_s=(0.0, 0.0, 0.0)
                ),
                propagate_covariance=True
            )
    
    tracker.update_map()
    for h in tracker.hypotheses:
        # If it was a map match this round, how do we know? We don't track it explicitly,
        # but we know baseline's score is simply scaled.
        print(f"  {h.id}: {h.parent_segment_id} | Score: {h.score:.3f} | Z-pos: {h.ins.state.position_m[2]:.2f}m | Z-cov: {h.ins.covariance[2,2]:.2f}")

    print("\n--- Time 10.0s: Resolving on Flyover ---")
    for i in range(500, 1000):
        for h in tracker.hypotheses:
            h.ins.propagate(
                ImuSample(
                    timestamp_ns=int((i+1) * 1e7),
                    accel_m_s2=(0.0, 0.0, -9.80665), # Level flight
                    gyro_rad_s=(0.0, 0.0, 0.0)
                ),
                propagate_covariance=True
            )
            
    # Inject a weak barometer or GNSS altitude measurement showing we are definitely up
    # Direct ESKF injection
    for h in tracker.hypotheses:
        # Mahalanobis dist calculation from a single Z measurement
        H = np.zeros((1, 15))
        H[0, 2] = 1.0 # Z position
        z_err = -14.0 - h.ins.state.position_m[2] # We measure -14m altitude
        h.eskf._apply_measurement(np.array([z_err]), H, np.array([[2.0]]), mahalanobis_gate=10.0)

    tracker.update_map()
    for h in tracker.hypotheses:
        # If it was a map match this round, how do we know? We don't track it explicitly,
        # but we know baseline's score is simply scaled.
        print(f"  {h.id}: {h.parent_segment_id} | Score: {h.score:.3f} | Z-pos: {h.ins.state.position_m[2]:.2f}m | Z-cov: {h.ins.covariance[2,2]:.2f}")

if __name__ == "__main__":
    run_flyover_simulation()
