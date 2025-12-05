import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation
from typing import List

from pyvio.core.domain.samples import SensorType
from .stage import Stage


class LiveVisualizer:
    def __init__(self, stage: Stage, sensors: List[SensorType], maxlen=1000):
        self.stage = stage
        self._sensors = sensors
        self.buffers = {s: deque(maxlen=maxlen) for s in sensors}

        self.fig, self.axs = plt.subplots(len(sensors), 1, figsize=(8, 6))
        self.lines = {}

        for i, sensor in enumerate(sensors):
            self.lines[sensor] = [
                self.axs[i].plot([], [], label=axis)[0] for axis in "XYZ"
            ]
            self.axs[i].set_title(sensor.name)
            self.axs[i].legend()

        plt.ion()

        for s in self._sensors:
            self.stage.subscribe(s, lambda sample, s=s: self.buffers[s].append(sample))

    def update_plot(self, frame):
        for i, sensor in enumerate(self._sensors):
            window = list(self.buffers[sensor])
            if not window:
                continue

            times = [s.timestamp * 1e-9 for s in window]

            for j, axis in enumerate("XYZ"):
                values = [getattr(s, axis.lower()) for s in window]
                line = self.lines[sensor][j]
                line.set_data(times, values)
                ax = line.axes
                ax.set_xlim(times[0], times[-1])
                ax.relim()
                ax.autoscale_view(scaley=True)

            queue = self.stage.queues[sensor]
            interval = queue.mean("timestamp_diff") * 1e-9 + 1e-9
            stdev_ms = queue.stdev("timestamp_diff") * 1e-3
            self.axs[i].set_title(
                f"{sensor.name} | rate={1/interval:.1f} Hz stdev={stdev_ms:.3f} us"
            )

        return [l for lines in self.lines.values() for l in lines]

    def start_animation(self, interval=50):
        self.ani = FuncAnimation(
            self.fig, self.update_plot, frames=500, interval=interval, blit=False
        )
        plt.show(block=True)
