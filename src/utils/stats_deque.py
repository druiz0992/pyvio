from collections import deque
from threading import Lock
from operator import attrgetter
from typing import Generic, TypeVar, List
from enum import Enum, auto
from dataclasses import dataclass

from .stats import RollingStat

""" Use example

    class Sample:
      def __init__(self, timestamp, x,y,z):
          self.timestamp = timestamp
          self.x = x
          self.y = y
          self.z = z
          
    buf = StatsDeque(
        maxlen=100,
        window=20,
        stats = [
            StatsSpec.raw("x", "x"),
            StatsSpec.diff("timestamp_dipp", "timestamp")
        ]
    )
    
    for i in range(25):
        buf.append(Sample(i, i*2, i*3, i*4))
    
    print("Mean x:", buf.mean("x"))
    print("Stdev ts:", buf.stdev("timestamp_diff"))
"""


class StatOp(Enum):
    RAW = auto()
    DIFF = auto()


@dataclass
class StatSpec:
    name: str
    field: str
    op: StatOp

    @staticmethod
    def raw(name: str, field: str):
        return StatSpec(name, field, StatOp.RAW)

    @staticmethod
    def diff(name: str, field: str):
        return StatSpec(name, field, StatOp.DIFF)


TIMESTAMP_DIFF = StatSpec.diff("timestamp_diff", "timestamp")


T = TypeVar("T")


class StatsQueue(Generic[T]):
    """Queue of samples T + incremental rolling stats on selected attributes.

    Args:
        maxlen: Maximum total samples stored (the raw buffer).
        window: Rolling window size for statistics.
        stats: List of attribute names of T for which stats are computed.

    NOTES:
      I maintain a separate buffer for stats so that they are independent from stored samples.
      That is, i can pop samples and not worry about stats.
    """

    def __init__(self, maxlen: int, window: int, stats: List[StatSpec]):
        self.data: deque[T] = deque(maxlen=maxlen)
        self.lock = Lock()

        self.stats_config = {spec.name: (spec.field, spec.op) for spec in stats}

        self.accessors = {
            stat_name: attrgetter(attr)
            for stat_name, (attr, _) in self.stats_config.items()
        }

        self.stats = {name: RollingStat(window) for name in self.stats_config}

    def put(self, sample: T):
        with self.lock:
            prev = self.data[-1] if len(self.data) > 0 else None

            for stat_name, (field, op) in self.stats_config.items():
                accessor = self.accessors[stat_name]

                if op == StatOp.RAW:
                    value = accessor(sample)
                    self.stats[stat_name].add(value)

                elif op == StatOp.DIFF:
                    if prev is None:
                        value = 0.0
                    else:
                        value = accessor(sample) - accessor(prev)
                    self.stats[stat_name].add(value)
                else:
                    raise ValueError(f"Unknown StatOp: {op}")

            self.data.append(sample)

    def get(self) -> T | None:
        with self.lock:
            if not self.data:
                return None
            return self.data.popleft()

    def mean(self, name: str) -> float:
        with self.lock:
            return self.stats[name].mean()

    def stdev(self, name: str) -> float:
        with self.lock:
            return self.stats[name].stdev()

    def __len__(self):
        return len(self.data)

    def __iter__(self):
        with self.lock:
            return iter(list(self.data))
