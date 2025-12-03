import gpiod
from gpiod.line import Direction, Value, Edge, Clock
from gpiod.edge_event import EdgeEvent
import time

CHIP = "gpiochip0"
LINE_OFFSET = 4  # GPIO4
CONSUMER = "pps_test"

chip = gpiod.Chip(CHIP)

print(f"Monitoring line {LINE_OFFSET} on {CHIP} for PPS pulses.")
with gpiod.request_lines(
    "/dev/gpiochip0",
    consumer="blink-example",
    config={
        LINE_OFFSET: gpiod.LineSettings(
            direction=Direction.INPUT,
            edge_detection=Edge.BOTH,
            event_clock=Clock.REALTIME,
        )
    },
) as request:
    while True:
        if request.wait_edge_events(1):
            ev = request.read_edge_events()[0]
            t = ev.timestamp_ns  # float seconds
            if ev.event_type == EdgeEvent.event_type.RISING_EDGE:
                if last_rising_time is not None:
                    freq = 1.0 / (t - last_rising_time)
                    print(f"Rising edge. Frequency: {freq:.2f} Hz")
                else:
                    print("First rising edge detected")
                last_rising_time = t
                rising_time = t
            elif ev.event_type == EdgeEvent.event_type.FALLING_EDGE:
                if "rising_time" in locals():
                    pulse_width = t - rising_time
                    print(f"Falling edge. Pulse width: {pulse_width*1e6:.2f} µs")
