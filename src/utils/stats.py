import math
from collections import deque
from enum import Enum
from dataclasses import dataclass


class RollingStat:
    """Incremental rolling statistics using a fixed size circular buffer."""

    def __init__(self, window: int):
        self.window = window
        self.buf = deque(maxlen=window)
        self.sum = 0.0
        self.sum_sq = 0.0

    def add(self, value: float):
        if len(self.buf) == self.window:
            old = self.buf[0]
            self.sum -= old
            self.sum_sq -= old * old

        self.buf.append(value)
        self.sum += value
        self.sum_sq += value * value

    def mean(self) -> float:
        if not self.buf:
            return 0.0
        return self.sum / len(self.buf)

    def stdev(self) -> float:
        n = len(self.buf)
        if n < 2:
            return 0.0
        mean = self.mean()
        variance = max(0.0, (self.sum_sq - n * mean**2) / (n - 1))
        return math.sqrt(variance)
