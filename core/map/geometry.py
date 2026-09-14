"""
Data structures for map geometry representation.
"""
from dataclasses import dataclass
import numpy as np
from typing import Tuple

@dataclass
class RoadSegment:
    """
    A single directed line segment representing a road in the map.
    Coordinates are defined in the local Navigation Frame (NED).
    """
    id: str
    start_ned: Tuple[float, float, float]
    end_ned: Tuple[float, float, float]
    has_elevation: bool = False  # Track if segment has explicit vertical constraints
    
    @property
    def vector_2d(self) -> np.ndarray:
        return np.array([self.end_ned[0] - self.start_ned[0], self.end_ned[1] - self.start_ned[1]])
        
    @property
    def length_2d(self) -> float:
        return float(np.linalg.norm(self.vector_2d))
        
    @property
    def unit_direction_2d(self) -> np.ndarray:
        length = self.length_2d
        if length < 1e-6:
            return np.array([1.0, 0.0])
        return self.vector_2d / length
