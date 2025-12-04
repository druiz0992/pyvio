from .pps import PPSMonitor
from .unwrap import TimestampUnwrapper
from .estimator import Estimator
from core.domain.samples import RawSensorSample, SensorType
from core.domain.sync_params import SyncParams


class Clock:
    """Clock providing timestamp services."""

    def __init__(self, clock_freq_hz: int, sync_params: SyncParams):
        self._clock_freq_hz = clock_freq_hz
        self._ticks_ns: int = 1_000_000_000 // self._clock_freq_hz
        self._ticks_rem: int = 1_000_000_000 % self._clock_freq_hz
        self._unwrapper = TimestampUnwrapper()

        self._estimator = Estimator()

        self._pps = PPSMonitor(sync_params.gpio)
        self._pps.start()
        self._pps.subscribe(self._estimator.filter)
        

    def get_timestamp(self, sensor: SensorType, raw_ts: int) -> int:
        ts = self._unwrapper.unwrap(sensor, raw_ts)
        timestamp = ts * self._ticks_ns + (ts * self._ticks_rem) // self._clock_freq_hz
        
        if sensor == SensorType.TIMER:
            return timestamp
        else:
            drift, offset = self._estimator.get_params()
            return int(drift * timestamp + offset)
    
    def update_mcu_timestamp(self, sample: RawSensorSample):
        self._estimator.set_mcu(sample.timestamp, sample.x)
    