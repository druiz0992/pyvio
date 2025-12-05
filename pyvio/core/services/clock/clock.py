from .pps import PPSMonitor
from .unwrap import TimestampUnwrapper
from .estimator import Estimator
from pyvio.core.domain.samples import RawSensorSample, SensorType
from pyvio.core.domain.sync_params import SyncParams


class Clock:
    """Clock providing timestamp services."""

    def __init__(self, sync_params: SyncParams):
        self._unwrapper = TimestampUnwrapper(sync_params.clock_freq_hz)

        self._estimator = Estimator()

        self._pps = PPSMonitor(sync_params.gpio, self._estimator.filter)
        

    def get_timestamp(self, sensor: SensorType, raw_ts: int) -> int:
        timestamp = self._unwrapper.unwrap(sensor, raw_ts)
        
        if sensor == SensorType.TIMER:
            return timestamp
        else:
            drift, offset = self._estimator.get_params()
            return int(drift * timestamp + offset)
    
    def update_mcu_timestamp(self, sample: RawSensorSample):
        self._estimator.set_mcu(sample.timestamp, sample.x)
    