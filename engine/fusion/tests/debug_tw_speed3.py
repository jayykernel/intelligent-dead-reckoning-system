from engine.fusion.tests.debug_tw_speed2 import run_sweep

for s in [0.35, 0.40, 0.45, 0.50, 0.60, 0.70]:
    err, drift = run_sweep(s)
    print(f"Scale: {s:.2f} | Pos Err: {err:.2f} m | Drift %: {drift:.2f} %")
