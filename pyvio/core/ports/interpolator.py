from __future__ import annotations
from abc import ABC, abstractmethod
from collections import deque
from typing import Optional
import numpy as np

from pyvio.core.ports.sample import SamplePort


class IMUInterpolator(ABC):
    """
    Abstract base class for IMU interpolation (accelerometer + gyro).
    """

    def __init__(self, accel_buf: deque, gyro_buf: deque):
        self.accel_buf = accel_buf
        self.gyro_buf  = gyro_buf

    @abstractmethod
    def interpolate(self, buffer: deque, t_query: int) -> Optional[SamplePort]:
        """Return the interpolated vector at t_query."""
        pass
