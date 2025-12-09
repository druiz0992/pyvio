from abc import ABC, abstractmethod
from typing import Optional, List, Callable
from ..domain.sample import RawSensorSample, SensorSample
from pyvio.core.ports.sample import SampleType


class SensorPort(ABC):
    @abstractmethod
    def get(self, sensor: Optional[SampleType] = None) -> SensorSample | None:
        """
        Get a sample from the sensor port.

        Args:
            sensor: Optional sensor type to retrieve from. If None, return the next available sample.

        Returns:
            RawSensorSample
        """
        pass

    @abstractmethod
    def put(self, sample: RawSensorSample) -> None:
        """Append a sample to the port (for testing or simulation)."""
        pass

    @abstractmethod
    def start(self) -> None:
        """Start reading from the sensor."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop reading from the sensor."""
        pass

    @abstractmethod
    def subscribe(self, sensor: SampleType, callback: Callable[[SensorSample], None]):
        """Observers will be called with each processed SensorSample."""
        pass

    @abstractmethod
    def get_buffer(self, sensor: SampleType) -> List[SensorSample]:
        """Return a snapshot of the buffer for this sensor."""
        pass
