from typing import List, Callable, Optional
import threading
import time
import numpy as np

from pyvio.core.ports.sensor import MockSensorPort, SampleType
from pyvio.core.ports.sensor import SampleType
from pyvio.core.domain.sample import (
    SensorSample,
)
from pyvio.core.domain.pipeline.stage import Stage
from pyvio.utils.stats_deque import TIMESTAMP_DIFF



class MockSensor(MockSensorPort):
    """
    A generic mock sensor port that generates SensorSample instances
    from a given trajectory and allows subscribing/getting samples.
    """

    def __init__(self, trajectory, sensors: List[SampleType], dt: float = 0.01, maxlen: int = 1000, window: int = 50):
        """
        trajectory: object implementing evaluate(t) -> (position, velocity, yaw)
        sensors: list of SampleType this port generates
        dt: sampling interval in seconds
        """
        self.trajectory = trajectory
        self.dt = dt
        self._running_ = False
        self._time = 0.0

        self.stage = Stage[SensorSample](sensors=sensors, maxlen=maxlen, window=window, stats=[TIMESTAMP_DIFF])

        # Lock for thread safety
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._running_ = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running_ = False
        if self._thread:
            self._thread.join(timeout=0.1)

    def put(self, sample: SensorSample) -> None:
        """Append a sample to the Stage."""
        self.stage.put(sample.sensor, sample)

    def get(self, sensor: Optional[SampleType] = None) -> Optional[SensorSample]:
        if sensor is None:
            # return the first available sample from any sensor queue
            for s in self.stage.queues.keys():
                sample = self.stage.get(s)
                if sample is not None:
                    return sample
            return None
        else:
            return self.stage.get(sensor)


    def get_buffer(self, sensor: SampleType) -> List[SensorSample]:
        return self.stage.get_buffer(sensor)

    def subscribe(self, sensor: SampleType, callback: Callable[[SensorSample], None]):
        self.stage.subscribe(sensor, callback)

    def _loop(self):
        while self._running_:
            with self._lock:
                self._generate_samples(self._time)
                self._time += self.dt
            time.sleep(self.dt)  # remove if you want faster-than-real-time


    @staticmethod
    def _rotate_to_body_frame(vec: np.ndarray, yaw: float) -> np.ndarray:
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        return R.T @ vec


class MockIMUSensor(MockSensor):
    def __init__(self, trajectory, dt: float = 0.01, accel_noise: float = 0.02, gyro_noise: float = 0.001):
        super().__init__(trajectory, sensors=[SampleType.ACCELEROMETER, SampleType.GYROSCOPE], dt=dt)
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self._last_vel = None
        self._last_yaw = None
        self.gravity = np.array([0, 0, 9.81])

    def _generate_samples(self, t: float):
        pos, vel, yaw = self.trajectory.evaluate(t)

        # Accelerometer: finite difference of velocity + gravity
        if self._last_vel is None:
            acc_world = np.zeros(3)
        else:
            acc_world = (vel - self._last_vel) / self.dt
        self._last_vel = vel

        acc_body = self._rotate_to_body_frame(acc_world + self.gravity, yaw)
        acc_body += np.random.randn(3) * self.accel_noise

        gyro = np.zeros(3)
        if self._last_yaw is not None:
            gyro[2] = (yaw - self._last_yaw) / self.dt
        self._last_yaw = yaw
        gyro += np.random.randn(3) * self.gyro_noise

        self.put(SensorSample(SampleType.ACCELEROMETER, int(t*1e9), *acc_body))
        self.put(SensorSample(SampleType.GYROSCOPE, int(t*1e9), *gyro))


class MockGPSSensor(MockSensor):
    def __init__(self, trajectory, dt: float = 1.0, pos_noise: float = 1.0, vel_noise: float = 0.05):
        super().__init__(trajectory, sensors=[SampleType.GPS, SampleType.ODOMETRY], dt=dt)
        self.pos_noise = pos_noise
        self.vel_noise = vel_noise

    def _generate_samples(self, t: float):
        pos, vel, yaw = self.trajectory.evaluate(t)

        gps_pos = pos + np.random.randn(3) * self.pos_noise

        # Odometry: velocity in body frame
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        vel_body = R.T @ vel
        vel_body += np.random.randn(3) * self.vel_noise

        self.put(SensorSample(SampleType.GPS, int(t*1e9), *gps_pos))
        self.put(SensorSample(SampleType.ODOMETRY, int(t*1e9), *vel_body))