from .unwrap import TimestampUnwrapper
from core.domain.samples import SensorType


class Clock:
    """Clock providing timestamp services via composition of modules."""

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        self.unwrapper = TimestampUnwrapper()

    def get_timestamp(self, sensor: SensorType, raw_ts: int) -> float:
        ts = self.unwrapper.unwrap(sensor, raw_ts)
        return float(ts)
