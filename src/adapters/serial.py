import serial
import threading
from typing import List, Callable
from core.ports.sensor import SensorPort
from core.domain.samples import (
    RawSensorSample,
    SensorType,
    SensorSample,
    SampleEncoding,
)
from core.config import Config
from core.services.clock.clock import Clock
from core.domain.pipeline.stage import Stage
from utils.stats_deque import TIMESTAMP_DIFF


class SerialSensorAdapter(SensorPort):
    def __init__(
        self,
        cfg: Config,
        maxlen: int = 1000,
        window: int = 50,
    ):
        self.ser = serial.Serial(cfg.serial_port_name, cfg.serial_port_rate, timeout=1)
        self.encoding = cfg.sample_encoding
        self.sensor_sensitivity = cfg.sensor_sensitivity()
        self.sample_width = cfg.sensor_sample_width()
        self._running_ = True
        self.clock = Clock(cfg.imu_sync())

        self.stage = Stage[SensorSample](
            sensors=SensorType.imu_list(),
            maxlen=maxlen,
            window=window,
            stats=[TIMESTAMP_DIFF],
        )
        
    def stop(self):
        self._running_ = False

    def start(self):
        t = threading.Thread(target=self._reader_loop, daemon=True)
        t.start()

    def put(self, sample: RawSensorSample):
        """Append sample to the corresponding sensor queue."""
        if sample.sensor != SensorType.TIMER:
            processed_sample = self._apply_sensitivity(sample)
            self.stage.put(sample.sensor, processed_sample)
        else:
            self.clock.update_mcu_timestamp(sample)

    def get(self, sensor: SensorType) -> SensorSample | None:
        """Get oldest sample from specific sensor queue."""
        return self.stage.get(sensor)

    def get_buffer(self, sensor: SensorType) -> List[SensorSample]:
        """Return a snapshot of the internal queue."""
        return self.stage.get_buffer(sensor)

    def subscribe(self, sensor: SensorType, callback: Callable[[SensorSample], None]):
        """Observers will be called with each processed SensorSample."""
        self.stage.subscribe(sensor, callback)

    def _reader_loop(self):
        if self.encoding == SampleEncoding.ASCII:
            while self._running_:
                line = self.ser.readline()
                if not line:
                    continue
                line = line.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                try:
                    sample = RawSensorSample.from_str(
                        line,
                        self.clock,
                    )
                except ValueError:
                    continue

                self.put(sample)

        elif self.encoding == SampleEncoding.BINARY:
            sample_size = RawSensorSample.SENSOR_SAMPLE_SIZE

            while self._running_:
                # Synchronize chunk with beginning of sensor data (sensor type)
                code = self.ser.read(1)
                if not code or (sensor_type := SensorType.from_binary(code[0])) is None:
                    continue

                data = self.ser.read(sample_size - 1)

                if len(data) != sample_size - 1:
                    continue

                sample = RawSensorSample.from_bytes(
                    sensor_type,
                    data,
                    self.clock,
                )

                self.put(sample)

    def _apply_sensitivity(self, sample: RawSensorSample) -> SensorSample:
        N = self.sample_width[sample.sensor]
        right_shift = 16 - N
        factor = self.sensor_sensitivity[sample.sensor]

        def convert(raw):
            val = raw >> right_shift
            if val & (1 << (N - 1)):
                val -= 1 << N
            return val * factor

        return SensorSample(
            sensor=sample.sensor,
            timestamp=sample.timestamp,
            x=convert(sample.x),
            y=convert(sample.y),
            z=convert(sample.z),
        )
