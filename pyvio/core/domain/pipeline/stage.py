from typing import Callable, Dict, Generic, TypeVar, List, Optional

from pyvio.core.domain.samples import SensorType
from pyvio.utils.stats_deque import StatsQueue, StatSpec, TIMESTAMP_DIFF

T = TypeVar("T")


class Stage(Generic[T]):
    """Stage handling multiple sensor queues with optional observers."""

    def __init__(
        self,
        sensors: List[SensorType],
        maxlen: int = 1000,
        window: int = 50,
        stats: List[StatSpec] = [TIMESTAMP_DIFF],
    ):
        self.queues: Dict[SensorType, StatsQueue[T]] = {
            s: StatsQueue(maxlen=maxlen, window=window, stats=stats) for s in sensors
        }
        self.observers: Dict[SensorType, List[Callable[[T], None]]] = {
            s: [] for s in sensors
        }

    def subscribe(self, sensor: SensorType, callback: Callable[[T], None]):
        self.observers[sensor].append(callback)

    def put(self, sensor: SensorType, sample: T):
        if sensor not in self.queues:
            raise ValueError(f"No queue for sensor {sensor}")
        self.queues[sensor].put(sample)
        for cb in self.observers[sensor]:
            try:
                cb(sample)
            except Exception:
                pass

    def get(self, sensor: SensorType) -> Optional[T]:
        if sensor not in self.queues:
            raise ValueError(f"No queue for sensor {sensor}")
        return self.queues[sensor].get()

    def get_buffer(self, sensor: SensorType) -> List[T]:
        """Return a snapshot of the internal queue for read-only access."""
        if sensor not in self.queues:
            return []
        return list(self.queues[sensor].data)
