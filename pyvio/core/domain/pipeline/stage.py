from typing import Callable, Dict, Generic, TypeVar, List, Optional

from pyvio.core.ports.sample import SampleType
from pyvio.utils.stats_deque import StatsQueue, StatSpec, TIMESTAMP_DIFF

T = TypeVar("T")


class Stage(Generic[T]):
    """Stage handling multiple sensor queues with optional observers."""

    def __init__(
        self,
        sensors: List[SampleType],
        maxlen: int = 1000,
        window: int = 50,
        stats: List[StatSpec] = [TIMESTAMP_DIFF],
    ):
        self.queues: Dict[SampleType, StatsQueue[T]] = {
            s: StatsQueue(maxlen=maxlen, window=window, stats=stats) for s in sensors
        }
        self.observers: Dict[SampleType, List[Callable[[T], None]]] = {
            s: [] for s in sensors
        }

    def subscribe(self, sensor: SampleType, callback: Callable[[T], None]):
        self.observers[sensor].append(callback)

    def put(self, sensor: SampleType, sample: T):
        if sensor not in self.queues:
            raise ValueError(f"No queue for sensor {sensor}")
        self.queues[sensor].put(sample)
        for cb in self.observers[sensor]:
            try:
                cb(sample)
            except Exception:
                pass

    def get(self, sensor: SampleType) -> Optional[T]:
        if sensor not in self.queues:
            raise ValueError(f"No queue for sensor {sensor}")
        return self.queues[sensor].get()

    def get_buffer(self, sensor: SampleType) -> List[T]:
        """Return a snapshot of the internal queue for read-only access."""
        if sensor not in self.queues:
            return []
        return list(self.queues[sensor].data)
