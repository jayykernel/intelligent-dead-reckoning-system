import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from training.data_loader import load_iovnbd_session

data_dir = "data/raw"
driver = "S (Driver A)"
session = "S4"

try:
    s_df, v_df = load_iovnbd_session(data_dir, driver, session)
    print(f"SUCCESS: Loaded {driver} / {session}")
    # The loader defines session_dir internally, let's print it to see what it found
    # I'll just temporarily add a print in the loader, but I can infer it:
    
    # Path construction in loader:
    possible_paths = [
        os.path.join(data_dir, driver, session),
        os.path.join(data_dir, "Categorised IOVNB Dataset", driver, session)
    ]
    for path in possible_paths:
        if os.path.exists(path):
            print(f"Path FOUND: {path}")
            break
except Exception as e:
    print(f"ERROR: {e}")

# Check classification path
class_model_path = "training/models/vehicle_classifier.tflite"
print(f"Classifier model exists: {os.path.exists(class_model_path)}")
