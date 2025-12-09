from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Self
import numpy as np
from enum import Enum

class SampleType(Enum):
    ACCELEROMETER = "A"
    GYROSCOPE = "G"
    MAGNETOMETER = "M"
    TIMER = "T"
    GPS = "P"
    ODOMETRY = "O"
    IMU = "I"
    STATE = "S"

    @classmethod
    def from_binary(cls, code: int) -> SampleType | None:
        mapping = {
            0: cls.MAGNETOMETER,
            1: cls.ACCELEROMETER,
            2: cls.GYROSCOPE,
            3: cls.TIMER,
            4: cls.GPS,
            5: cls.ODOMETRY,
            6: cls.IMU,
            7: cls.STATE
        }
        return mapping.get(code)

    def to_binary(self) -> int:
        """Return the integer code corresponding to this SampleType."""
        mapping = {
            self.MAGNETOMETER: 0,
            self.ACCELEROMETER: 1,
            self.GYROSCOPE: 2,
            self.TIMER: 3,
            self.GPS: 4,
            self.ODOMETRY: 5,
            self.IMU: 6,
            self.STATE: 7
        }
        if self not in mapping:
            raise ValueError(f"SampleType {self} cannot be serialized to binary code")
        return mapping[self]

    @classmethod
    def list(cls) -> list["SampleType"]:
        """Return a list of all SampleType variants."""
        return list(cls)

    @classmethod
    def imu_list(cls) -> list["SampleType"]:
        """Return a list of IMU SampleType variants."""
        return [
            cls.ACCELEROMETER,
            cls.GYROSCOPE,
            cls.MAGNETOMETER,
            cls.TIMER,
        ]

    @classmethod
    def interp_list(cls) -> list["SampleType"]:
        """Return a list of IMU SampleType variants."""
        return [
            cls.ACCELEROMETER,
            cls.GYROSCOPE,
        ]
        
    @classmethod
    def sensor_list(cls) -> list["SampleType"]:
        """Return a list of Sensor SampleType variants."""
        return [
            cls.ACCELEROMETER,
            cls.GYROSCOPE,
            cls.MAGNETOMETER,
            cls.TIMER,
            cls.GPS,
            cls.ODOMETRY
        ]
        
    @classmethod
    def sample_list(cls) -> list["SampleType"]:
        """Return a list of Sensor SampleType variants."""
        return [
            cls.ACCELEROMETER,
            cls.GYROSCOPE,
            cls.MAGNETOMETER,
            cls.TIMER,
            cls.GPS,
            cls.ODOMETRY,
            cls.IMU,
            cls.STATE
        ]
        
    @classmethod
    def gps_list(cls) -> list["SampleType"]:
        """Return a list of GPS SampleType variants."""
        return [cls.GPS, cls.ODOMETRY]
    
    @classmethod
    def state_list(cls) -> list["SampleType"]:
        """Return a list of State SampleType variants."""
        return [
            cls.STATE,
        ]
        
        
class SamplePort(ABC):
    @abstractmethod
    def as_array(self) -> np.ndarray:
        pass

    @abstractmethod
    def to_bytes(self) -> bytes:
        """ Serialize the sample to bytes using struct."""
        pass

    @classmethod
    @abstractmethod
    def sample_size(cls) -> int:
        """Return the number of bytes required to serialize one sample."""
        pass
    

    @classmethod
    @abstractmethod
    def from_bytes(cls, data: bytes) -> Self:
        pass
    
    @classmethod
    @abstractmethod
    def sample_type(cls) -> SampleType:
        pass