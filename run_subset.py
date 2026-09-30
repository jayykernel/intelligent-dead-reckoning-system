from eval.run_full_benchmark import evaluate_dead_reckoning_session

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}
evaluate_dead_reckoning_session(cfg)

cfg2 = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta29"}
evaluate_dead_reckoning_session(cfg2)
