import gpiod
from gpiod.line import Direction, Edge, Clock
from core.ports.pps import PulsePerSecondPort
from core.domain.sync_params import GPIO
from typing import Callable
import time


class PulsePerSecondGPIO(PulsePerSecondPort):
    def __init__(
        self, line_offset: GPIO, chip_path="/dev/gpiochip0", edge=Edge.FALLING
    ):
        self.chip_path = chip_path
        self.line_offset = line_offset
        self.edge = edge
        self.clock = Clock.REALTIME
        self.handlers = []

    def subscribe(self, handler: Callable[[float], None]):
        """Subscribe a callback to be called on every edge event. Callback signature: handler(timestamp: float)"""
        self.handlers.append(handler)

    def _monitor_loop(self):
        if self.line_offset == GPIO.NONE:
            while True:
                time.sleep(0.01)
                timestamp = time.monotonic_ns() / 1e9
                for handler in self.handlers:
                    handler(timestamp)

        else:
            chip = gpiod.Chip(self.chip_path)
            with gpiod.request_lines(
                chip.path,
                config={
                    self.line_offset.value: gpiod.LineSettings(
                        direction=Direction.INPUT,
                        edge_detection=self.edge,
                        event_clock=self.clock,
                    )
                },
            ) as request:
                while True:
                    if request.wait_edge_events(timeout=1):
                        events = request.read_edge_events()
                        for ev in events:
                            if ev.event_type == gpiod.EdgeEvent.Type.FALLING_EDGE:
                                timestamp = ev.timestamp_ns / 1e9
                                for handler in self.handlers:
                                    handler(timestamp)
