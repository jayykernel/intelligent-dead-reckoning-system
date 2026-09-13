import pytest
from core.models.velocity_estimator import VelocityEstimatorAPI

def test_ml_velocity_api_update_rate():
    """Test that VelocityEstimatorAPI restricts the update rate to mitigate correlated residuals."""
    api = VelocityEstimatorAPI(model_path=None, window_size=10, update_interval=5)
    
    # Fill up the buffer first (need window_size samples + update_interval to be ready)
    for i in range(15):
        api.add_vehicle_frame_sample(
            accel_v=(1.0, 0.0, 0.0),
            gyro_v=(0.0, 0.0, 0.0)
        )
    
    assert api.should_update() == True, "Should be ready to update after reaching window_size and passing update interval"
    
    # Estimate velocity resets the counter
    api.estimate_velocity()
    
    # Even though we have a full buffer, immediately after an update it should not update again (throttled)
    assert api.should_update() == False, "Should be throttled immediately after an update"
    
    # Add fewer samples than the update interval
    for _ in range(3):
        api.add_vehicle_frame_sample(
            accel_v=(1.0, 0.0, 0.0),
            gyro_v=(0.0, 0.0, 0.0)
        )
        
    assert api.should_update() == False, "Should still be throttled before reaching update_interval"
    
    # Add enough samples to cross the designated update_interval
    for _ in range(2):
        api.add_vehicle_frame_sample(
            accel_v=(1.0, 0.0, 0.0),
            gyro_v=(0.0, 0.0, 0.0)
        )
        
    assert api.should_update() == True, "Should be ready to update again after waiting update_interval"
    
