import numpy as np
from eval.run_full_benchmark import evaluate_dead_reckoning_session, load_iovnbd_session, preprocess_session

cfg = {"category": "car", "driver": "Vta (Driver E)", "session": "Vta28"}

# Let's inspect the road segments and the map matcher choices
