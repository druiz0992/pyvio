from adapters.pps_gpio import PulsePerSecondGPIO
from utils.gpio import GPIO
from typing import Callable
import threading

class PPSMonitor():
    def __init__(self):
        self.pps = PulsePerSecondGPIO(GPIO.NONE)
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self.pps._monitor_loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()

    def subscribe(self, handler: Callable[[float], None]):
        """Subscribe a callback to be called on every edge event. Callback signature: handler(timestamp: float)"""
        self.pps.subscribe(handler)
