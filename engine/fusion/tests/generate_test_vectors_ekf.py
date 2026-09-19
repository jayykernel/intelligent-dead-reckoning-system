import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from engine.fusion.ekf import ErrorStateEKF

def generate():
    ekf = ErrorStateEKF(dt=0.1)

    # Set somewhat non-zero initial state
    q0 = np.array([0.9238795, 0.0, 0.0, 0.3826834]) # approx 45 deg yaw
    ekf.set_initial_state(
        p0=np.array([10.0, 20.0, 0.0]),
        v0=np.array([5.0, 0.0, 0.0]),
        q0=q0,
        b_a0=np.array([0.1, -0.05, 0.02]),
        b_g0=np.array([0.01, 0.02, -0.01])
    )

    out_states = []

    # Sequence of updates
    # 1. Predict
    ekf.predict(acc_raw=np.array([0.1, 9.8, 0.2]), gyro_raw=np.array([0.01, 0.02, 0.1]))
    out_states.append({"type": "predict", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    # 2. GNSS Pos
    ekf.update_gnss_position(p_gnss_enu=np.array([10.5, 20.0, 0.0]), sigma_pos=2.0)
    out_states.append({"type": "gnss_pos", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    # 3. GNSS Vel
    ekf.update_gnss_velocity(v_gnss_enu=np.array([4.9, -0.1, 0.0]))
    out_states.append({"type": "gnss_vel", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    # 4. Heading
    ekf.update_heading(heading_rad=0.785398) # ~45 deg
    out_states.append({"type": "heading", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    # 5. AI Speed
    ekf.update_ai_forward_speed(speed_fwd=12.0)
    out_states.append({"type": "ai_speed", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    # 6. ZUPT
    ekf.update_zupt()
    out_states.append({"type": "zupt", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    # 7. NHC
    ekf.update_nhc(vehicle_type="two_wheeler", lean_angle_rad=0.1)
    out_states.append({"type": "nhc", "p": ekf.p.tolist(), "v": ekf.v.tolist(), "q": ekf.q.tolist(), "ba": ekf.b_a.tolist(), "bg": ekf.b_g.tolist(), "P": ekf.P.tolist()})

    os.makedirs("engine/fusion/tests/vectors", exist_ok=True)
    with open("engine/fusion/tests/vectors/ekf_out.json", "w") as f:
        json.dump(out_states, f)

if __name__ == "__main__":
    generate()
