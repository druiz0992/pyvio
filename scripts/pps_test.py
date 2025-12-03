import gpiod
from gpiod.line import Direction, Value, Edge, Clock

CHIP = "/dev/gpiochip0"
LINE_OFFSET = 4  # GPIO4

chip = gpiod.Chip(CHIP)

last_rising_time = None
rising_time = None

print(f"Monitoring line {LINE_OFFSET} on {CHIP} for PPS pulses.")
with gpiod.request_lines(
    chip.path,
    config={
        LINE_OFFSET: gpiod.LineSettings(
            direction=Direction.INPUT,
            edge_detection=Edge.FALLING,
            event_clock=Clock.REALTIME,
        )
    },
) as request:
    while True:
        if request.wait_edge_events(timeout=1):
            events = request.read_edge_events()
            ev = events[0]
            
            t = ev.timestamp_ns/1e9  
            
            if ev.event_type == gpiod.EdgeEvent.Type.FALLING_EDGE:
                if last_rising_time is not None:
                    period = t - last_rising_time
                    freq = 1.0 / period
                    print(f"Frequency: {freq:.2f} Hz")

                last_rising_time = t
                rising_time = t
