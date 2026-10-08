import json
import os
import tempfile
import numpy as np
import pytest
from engine.calibration.calibrator import CalibrationEngine
from engine.calibration.online_mag_cal import OnlineMagnetometerCalibrator

def test_calibrator_persistence_roundtrip():
    ce = CalibrationEngine()
    ce.gyro_bias = np.array([0.01, -0.02, 0.005])
    ce.accel_bias = np.array([0.1, -0.1, 0.2])
    # Orthonormal matrix with det = 1
    ce.R_phone_to_veh = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    ce.is_calibrated = True
    ce.alignment_score = 0.95
    ce.mag_hard_iron = np.array([10.0, -10.0, 5.0])
    ce.mag_soft_iron = np.array([[1.0, 0.0, 0.0], [0.0, 1.1, 0.0], [0.0, 0.0, 0.9]])
    ce.mag_is_calibrated = True
    ce.mag_calibration_quality = 0.8

    with tempfile.TemporaryDirectory() as tmpdir:
        filepath = os.path.join(tmpdir, "calib.json")
        ce.save_to_json(filepath)

        ce_loaded = CalibrationEngine.load_from_json(filepath)

        assert ce_loaded.is_calibrated == ce.is_calibrated
        assert np.isclose(ce_loaded.alignment_score, ce.alignment_score)
        assert np.allclose(ce_loaded.gyro_bias, ce.gyro_bias)
        assert np.allclose(ce_loaded.accel_bias, ce.accel_bias)
        assert np.allclose(ce_loaded.R_phone_to_veh, ce.R_phone_to_veh)
        assert ce_loaded.mag_is_calibrated == ce.mag_is_calibrated
        assert np.isclose(ce_loaded.mag_calibration_quality, ce.mag_calibration_quality)
        assert np.allclose(ce_loaded.mag_hard_iron, ce.mag_hard_iron)
        assert np.allclose(ce_loaded.mag_soft_iron, ce.mag_soft_iron)

def test_calibrator_validation_corrupted_file():
    with tempfile.TemporaryDirectory() as tmpdir:
        filepath = os.path.join(tmpdir, "corrupted.json")
        with open(filepath, 'w') as f:
            f.write("{ invalid json ...")

        ce = CalibrationEngine.load_from_json(filepath)
        assert not ce.is_calibrated
        assert ce.alignment_score == 0.0
        assert np.allclose(ce.gyro_bias, np.zeros(3))
        assert np.allclose(ce.R_phone_to_veh, np.eye(3))

def test_calibrator_validation_nan_rejection():
    data = {
        "schema_version": "1.0",
        "is_calibrated": True,
        "alignment_score": 0.5,
        "gyro_bias": [np.nan, 0.0, 0.0],
        "accel_bias": [0.0, 0.0, 0.0],
        "R_phone_to_veh": np.eye(3).tolist(),
        "mag_is_calibrated": False,
        "mag_calibration_quality": 0.0,
        "mag_hard_iron": [0.0, 0.0, 0.0],
        "mag_soft_iron": np.eye(3).tolist()
    }
    ce = CalibrationEngine.from_dict(data)
    assert not ce.is_calibrated
    assert np.allclose(ce.gyro_bias, np.zeros(3))

def test_calibrator_validation_non_orthonormal_R():
    data = {
        "schema_version": "1.0",
        "is_calibrated": True,
        "alignment_score": 0.5,
        "gyro_bias": [0.0, 0.0, 0.0],
        "accel_bias": [0.0, 0.0, 0.0],
        "R_phone_to_veh": [[1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], # Not orthogonal
        "mag_is_calibrated": False,
        "mag_calibration_quality": 0.0,
        "mag_hard_iron": [0.0, 0.0, 0.0],
        "mag_soft_iron": np.eye(3).tolist()
    }
    ce = CalibrationEngine.from_dict(data)
    assert not ce.is_calibrated
    assert np.allclose(ce.R_phone_to_veh, np.eye(3))

def test_online_mag_cal_roundtrip():
    omc = OnlineMagnetometerCalibrator()
    omc.mag_hard_iron = np.array([5.0, -15.0, 25.0])
    omc.mag_soft_iron = np.array([[1.0, 0.05, 0.0], [0.05, 1.0, 0.0], [0.0, 0.0, 1.0]])
    omc.is_calibrated = True
    omc.calibration_quality = 0.75
    omc.sample_count = 120
    omc.field_strength = 45.0

    with tempfile.TemporaryDirectory() as tmpdir:
        filepath = os.path.join(tmpdir, "mag_calib.json")
        omc.save_calibration(filepath)

        omc_loaded = OnlineMagnetometerCalibrator.load_calibration(filepath)
        assert omc_loaded.is_calibrated == omc.is_calibrated
        assert omc_loaded.sample_count == omc.sample_count
        assert np.isclose(omc_loaded.calibration_quality, omc.calibration_quality)
        assert np.allclose(omc_loaded.mag_hard_iron, omc.mag_hard_iron)
        assert np.allclose(omc_loaded.mag_soft_iron, omc.mag_soft_iron)
        assert np.isclose(omc_loaded.field_strength, omc.field_strength)

def test_online_mag_cal_invalid_eigenvalues():
    data = {
        "schema_version": "1.0",
        "is_calibrated": True,
        "sample_count": 100,
        "calibration_quality": 0.8,
        "mag_hard_iron": [0.0, 0.0, 0.0],
        "mag_soft_iron": [[15.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], # Eigenvalue 15 > 10
        "field_strength": 48.0
    }
    omc = OnlineMagnetometerCalibrator.from_dict(data)
    assert not omc.is_calibrated
    assert np.allclose(omc.mag_soft_iron, np.eye(3))
