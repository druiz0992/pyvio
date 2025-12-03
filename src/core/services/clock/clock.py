from .pps import PPSMonitor
from .unwrap import TimestampUnwrapper
from core.domain.samples import SensorType


class Clock:
    """Clock providing timestamp services."""

    def __init__(self, clock_freq_hz: float, resync_freq_hz: float = 1.0):
        self._clock_freq_hz = clock_freq_hz
        self._ticks_ns = 1_000_000_000 // self._clock_freq_hz
        self._ticks_rem = 1_000_000_000 % self._clock_freq_hz
        self._resync_ticks = self._clock_freq_hz/resync_freq_hz
        self.unwrapper = TimestampUnwrapper()
        
        self.pps = PPSMonitor()
        self.pps.start()
        self.pps.subscribe(self.log)

    def get_timestamp(self, sensor: SensorType, raw_ts: int) -> float:
        ts = self.unwrapper.unwrap(sensor, raw_ts)
        timestamp = ts * self._ticks_ns + (ts * self._ticks_rem) // self._clock_freq_hz
        return float(timestamp)
    
    def log(self, timestamp:float):
        p_a = self.unwrapper.prev_sample(SensorType.ACCELEROMETER)
        p_g = self.unwrapper.prev_sample(SensorType.GYROSCOPE)
        p_m = self.unwrapper.prev_sample(SensorType.MAGNETOMETER)
        
        
        print(f"{timestamp}, {p_a}, {p_g}, {p_m}")
