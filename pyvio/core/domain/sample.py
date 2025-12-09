from __future__ import annotations
from dataclasses import dataclass
from typing import Self
from enum import Enum
import struct
import numpy as np

from pyvio.core.ports.sample import SamplePort, SampleType

class SampleEncoding(Enum):
    BINARY = 0
    ASCII = 1

    def to_str(self) -> str:
        return self.name


@dataclass
class RawSensorSample:
    sensor: SampleType
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
        sensor: SampleType,
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
            sensor_type = SampleType(sensor_char)
        except ValueError:
            raise ValueError(f"Unknown sensor type: {sensor_char}")

        # Convert numeric fields
        timestamp = clock.get_timestamp(sensor_type, int(ts_str))
        x = int(x_str)
        y = int(y_str)
        z = int(z_str)

        return RawSensorSample(sensor_type, timestamp, x, y, z)


@dataclass
class SensorSample(SamplePort):
    sensor: SampleType
    timestamp: int
    x: float
    y: float
    z: float

    STRUCT_FORMAT = "<HQfff"  # little-endian: H=uint16, Q=uint64, f=float32

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    @classmethod
    def from_raw(cls, sample: "RawSensorSample") -> "SensorSample":
        return cls(
            sensor=sample.sensor,
            timestamp=sample.timestamp,
            x=float(sample.x),
            y=float(sample.y),
            z=float(sample.z),
        )

    def to_bytes(self) -> bytes:
        """
        Serialize the sample to bytes using struct.
        Format:
        - sensor ID: unsigned short (2 bytes)
        - timestamp: unsigned long long (8 bytes)
        - x, y, z: float (4 bytes each)
        Total: 2 + 8 + 12 = 22 bytes
        """
        sensor_type = self.sensor.to_binary()
        return struct.pack(
            self.STRUCT_FORMAT, sensor_type, self.timestamp, self.x, self.y, self.z
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> Self:
        """
        Deserialize from bytes back to SensorSample.
        """
        sensor_val, timestamp, x, y, z = struct.unpack(cls.STRUCT_FORMAT, data)
        sensor_type = SampleType.from_binary(sensor_val)
        sensor = SampleType(sensor_type)
        return cls(sensor, timestamp, x, y, z)

    @classmethod
    def sample_size(cls) -> int:
        """Return the number of bytes required to serialize one sample."""
        return struct.calcsize(cls.STRUCT_FORMAT)
    
    @classmethod
    def sample_type(cls) -> SampleType:
        return cls.sensor


@dataclass
class IMUSample(SamplePort):
    sensor: SampleType
    timestamp: int
    acc: np.ndarray
    gyro: np.ndarray

    STRUCT_FORMAT = "<Q" + "fff" + "fff" 

    def as_array(self) -> np.ndarray:
        t_vec = self.timestamp
        acc_vec = self.acc
        gyro_vec = self.gyro
        return np.concatenate([t_vec, acc_vec, gyro_vec])

    def to_bytes(self) -> bytes:
        """
        Serialize the sample to bytes using struct.
        Format:
        - sensor ID: unsigned short (2 bytes)
        - timestamp: unsigned long long (8 bytes)
        - x, y, z: float (4 bytes each)
        Total: 2 + 8 + 12 = 22 bytes
        """
        sensor_type = self.sensor.to_binary()
        return struct.pack(
            self.STRUCT_FORMAT, sensor_type, self.timestamp, self.acc, self.gyro
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> Self:
        """
        Deserialize from bytes back to SensorSample.
        """
        sensor_val, timestamp, acc, gyro = struct.unpack(cls.STRUCT_FORMAT, data)
        sensor_type = SampleType.from_binary(sensor_val)
        sensor = SampleType(sensor_type)
        return cls(sensor, timestamp, acc, gyro)

    @classmethod
    def sample_size(cls) -> int:
        """Return the number of bytes required to serialize one sample."""
        return struct.calcsize(cls.STRUCT_FORMAT)
    
    @classmethod
    def sample_type(cls) -> SampleType:
        return cls.sensor
