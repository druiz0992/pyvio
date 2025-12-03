from typing import Dict
from core.domain.samples import SensorType


class TimestampUnwrapper:
    """Unwrap a u32 hardware timer that overflows every ~90 seconds, per sensor"""

    def __init__(self):
        # store previous raw timestamp for each sensor
        self.prev_raw: Dict[SensorType, int] = {}
        self.offset: Dict[SensorType, int] = {}

    def unwrap(self, sensor: SensorType, raw_ts: int) -> int:
        if sensor not in self.prev_raw:
            self.prev_raw[sensor] = raw_ts
            self.offset[sensor] = 0
            return raw_ts

        if raw_ts < self.prev_raw[sensor]:
            # overflow detected for this sensor
            self.offset[sensor] += 2**32

        self.prev_raw[sensor] = raw_ts
        return raw_ts + self.offset[sensor]
