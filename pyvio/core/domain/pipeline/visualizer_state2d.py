import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation
from typing import List
import numpy as np

from pyvio.core.ports.sample import SampleType
from .stage import Stage


class LiveVisualizerState2D:
    def __init__(self, stage: Stage, maxlen=1000):
        self.stage = stage
        self.buffer = deque(maxlen=maxlen)

        self.fig, (self.ax_pos, self.ax_vel) = plt.subplots(2, 1, figsize=(8, 8))

        # position plot
        (self.pos_line,) = self.ax_pos.plot([], [], "b.-", label="Position")
        (self.heading_line,) = self.ax_pos.plot([], [], "r-", label="Heading")
        self.ax_pos.set_xlabel("X [m]")
        self.ax_pos.set_ylabel("Y [m]")
        self.ax_pos.legend()
        self.ax_pos.axis("equal")

        # velocity plot
        (self.vx_line,) = self.ax_vel.plot([], [], "g-", label="Vx")
        (self.vy_line,) = self.ax_vel.plot([], [], "m-", label="Vy")
        self.ax_vel.set_xlabel("Time [s]")
        self.ax_vel.set_ylabel("Velocity [m/s]")
        self.ax_vel.legend()

        plt.ion()

        self.stage.subscribe(
            SampleType.STATE, lambda sample: self.buffer.append(sample)
        )

    def update_plot(self, frame):
        window = list(self.buffer)
        if not window:
            return []

        xs = [s.p[0] for s in window]
        ys = [s.p[1] for s in window]
        self.pos_line.set_data(xs, ys)
        print(f"pos: {xs} {ys}")

        # heading as small vector
        headings = [
            s.q.apply(np.array([1.0, 0.0, 0.0]))[:2] for s in window
        ]  # project to XY
        hx = [x + h[0] * 0.2 for x, h in zip(xs, headings)]
        hy = [y + h[1] * 0.2 for y, h in zip(ys, headings)]
        self.heading_line.set_data(hx, hy)

        self.ax_pos.relim()
        self.ax_pos.autoscale_view()

        # velocities
        times = [(s.timestamp - window[0].timestamp) * 1e-9 for s in window]
        vx = [s.v[0] for s in window]
        vy = [s.v[1] for s in window]
        self.vx_line.set_data(times, vx)
        self.vy_line.set_data(times, vy)
        self.ax_vel.relim()
        self.ax_vel.autoscale_view()

        return [self.pos_line, self.heading_line, self.vx_line, self.vy_line]

    def start_animation(self, interval=50):
        self.ani = FuncAnimation(
            self.fig, self.update_plot, frames=500, interval=interval, blit=False
        )
        plt.show(block=True)
