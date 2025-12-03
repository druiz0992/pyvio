from .pps import PPSMonitor
from .unwrap import TimestampUnwrapper
from core.domain.samples import SensorType
from core.domain.sync_params import SyncParams
from utils.kalman.simple import KalmanFilter
import numpy as np


class Clock:
    """Clock providing timestamp services."""

    def __init__(self, clock_freq_hz: int, sync_params: SyncParams):
        self._clock_freq_hz = clock_freq_hz
        self._ticks_ns: int = 1_000_000_000 // self._clock_freq_hz
        self._ticks_rem: int = 1_000_000_000 % self._clock_freq_hz
        self._unwrapper = TimestampUnwrapper()

        self._pps = PPSMonitor(sync_params.gpio)
        self._pps.start()
        self._pps.subscribe(self.filter)

        self._prev_mcu_t = 0
        self._nominal_resync_ns = 1 / sync_params.frequency_hz * 1e9
        self._upper_resync_ns = 1.1 * self._nominal_resync_ns
        self._lower_resync_ns = 0.9 * self._nominal_resync_ns
        # state [ drift, offset ]; host_t = mcu_t * drift + offset
        self._ekf = KalmanFilter(
            np.array([1.0, 0.0]), np.eye(2) * 1e-3, np.eye(2), np.eye(2) * 1e-6
        )

    def get_timestamp(self, sensor: SensorType, raw_ts: int) -> int:
        ts = self._unwrapper.unwrap(sensor, raw_ts)
        timestamp = ts * self._ticks_ns + (ts * self._ticks_rem) // self._clock_freq_hz
        return timestamp

    def filter(self, host_t: float):
        p_t = self._unwrapper.prev_sample(SensorType.TIMER)
        mcu_t = p_t * self._ticks_ns + (p_t * self._ticks_rem) // self._clock_freq_hz

        if (
            mcu_t - self._prev_mcu_t > self._lower_resync_ns
            and mcu_t - self._prev_mcu_t < self._upper_resync_ns
        ):
            H = np.array([[mcu_t, 1.0]])
            R = np.array([[1e-4]])
            self._ekf.predict()
            self._ekf.update(z=[host_t], H=H, R=R)
            print(f"{self._ekf.x[0], self._ekf.x[1]}")

        self._prev_mcu_t = mcu_t
