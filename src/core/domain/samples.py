from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import numpy as np


class SensorType(Enum):
    ACCELEROMETER = "A"
    GYROSCOPE = "G"
    MAGNETOMETER = "M"
    TIMER = "T"

    @classmethod
    def from_binary(cls, code: int) -> SensorType | None:
        mapping = {
            0: cls.MAGNETOMETER,
            1: cls.ACCELEROMETER,
            2: cls.GYROSCOPE,
            3: cls.TIMER,
        }
        return mapping.get(code)

    @classmethod
    def list(cls) -> list["SensorType"]:
        """Return a list of all SensorType variants."""
        return list(cls)


class SampleEncoding(Enum):
    BINARY = 0
    ASCII = 1

    def to_str(self) -> str:
        return self.name


@dataclass
class RawSensorSample:
    sensor: SensorType
    timestamp: int
    x: int
    y: int
    z: int

    TIMESTAMP_SLICE = slice(0, 4)
    X_SLICE = slice(4, 6)
    Y_SLICE = slice(6, 8)
    Z_SLICE = slice(8, 10)
    SENSOR_SAMPLE_SIZE = 11  # 1 byte sensor, 4 bytes timestamp, 3x2 bytes xyz

    @classmethod
    def from_bytes(
        cls,
        sensor: SensorType,
        data: bytes,
        clock,
    ) -> RawSensorSample:
        raw_ts = int.from_bytes(data[cls.TIMESTAMP_SLICE], "little")
        timestamp = clock.get_timestamp(sensor, raw_ts)

        x = int.from_bytes(data[cls.X_SLICE], "little", signed=False)
        y = int.from_bytes(data[cls.Y_SLICE], "little", signed=False)
        z = int.from_bytes(data[cls.Z_SLICE], "little", signed=False)

        return cls(sensor, timestamp, x, y, z)

    @classmethod
    def from_str(
        cls,
        data: str,
        clock,
    ) -> RawSensorSample:
        parts = data.strip().split(",")
        if len(parts) != 5:
            raise ValueError(f"Invalid line, expected 5 fields: {data}")

        sensor_char, ts_str, x_str, y_str, z_str = parts

        try:
            sensor_type = SensorType(sensor_char)
        except ValueError:
            raise ValueError(f"Unknown sensor type: {sensor_char}")

        # Convert numeric fields
        timestamp = clock.get_timstamp(sensor_type, int(ts_str))
        x = int(x_str)
        y = int(y_str)
        z = int(z_str)

        return RawSensorSample(sensor_type, timestamp, x, y, z)


@dataclass
class SensorSample:
    sensor: SensorType
    timestamp: int
    x: float
    y: float
    z: float

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)
