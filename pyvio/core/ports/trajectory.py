import numpy as np
from abc import ABC, abstractmethod

class TrajectoryPort(ABC):
    def __init__(self, radius: float = 5.0, height: float = 5.0, speed: float = 1.0):
        self.radius = radius
        self.height = height
        self.speed = speed

    @abstractmethod
    def evaluate(self, t: float) -> tuple[np.ndarray, np.ndarray, float]:
        """
        Compute position, velocity, and yaw at time t.
        Returns:
            position (np.ndarray[3]),
            velocity (np.ndarray[3]),
            yaw (float)
        """
        pass
