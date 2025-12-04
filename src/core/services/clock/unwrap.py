from typing import Dict
from core.domain.samples import SensorType


class TimestampUnwrapper:
    """Unwrap a u32 hardware timer that overflows every ~90 seconds, per sensor"""

    def __init__(self, clock_freq_hz: int):
        # store previous raw timestamp for each sensor
        self.prev_raw: Dict[SensorType, int] = {}
        self.offset: Dict[SensorType, int] = {}
        
        self._clock_freq_hz = clock_freq_hz
        if clock_freq_hz == 0:
            self._ticks_ns: int = 0
            self._ticks_rem: int = 0
        else:
            self._ticks_ns: int = 1_000_000_000 // self._clock_freq_hz
            self._ticks_rem: int = 1_000_000_000 % self._clock_freq_hz

    def unwrap(self, sensor: SensorType, raw_ts: int) -> int:
        if sensor not in self.prev_raw:
            self.prev_raw[sensor] = raw_ts
            self.offset[sensor] = 0
            return raw_ts

        if raw_ts < self.prev_raw[sensor]:
            # overflow detected for this sensor
            self.offset[sensor] += 2**32

        self.prev_raw[sensor] = raw_ts
        ts = raw_ts + self.offset[sensor]
        timestamp = ts * self._ticks_ns + (ts * self._ticks_rem) // self._clock_freq_hz
        return timestamp

