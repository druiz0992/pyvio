from abc import ABC, abstractmethod
from typing import Callable


class PulsePerSecondPort(ABC):
    @abstractmethod
    def subscribe(self, handler: Callable[[float], None]):
        """Subscribe a callback to be called on every edge event. Callback signature: handler(timestamp: float)"""
        pass

    @abstractmethod
    def _monitor_loop(self):
        """ wait for event"""
        pass