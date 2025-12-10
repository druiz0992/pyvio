import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np

from pyvio.core.ports.sample import SampleType
from .stage import Stage


class LiveVisualizerState2D:
    def __init__(self, stage: Stage, maxlen=2000):
        self.stage = stage
        self.buffer = deque(maxlen=maxlen)

        # --- Figure with grid layout ---
        self.fig = plt.figure(figsize=(14, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[2, 1])

        # --- Left: 2D POSE PLOT ---
        self.ax_pose = self.fig.add_subplot(gs[:, 0])
        self.ax_pose.set_xlabel("X [m]")
        self.ax_pose.set_ylabel("Y [m]")
        self.ax_pose.set_aspect("equal", adjustable="datalim")

        (self.path_line,) = self.ax_pose.plot([], [], color="0.6", linewidth=1)
        self.heading_history = []
        self.triangle = Polygon([[0, 0]], closed=True, color="blue")
        self.ax_pose.add_patch(self.triangle)

        # --- Right column: Timeseries plots ---
        self.ax_p = self.fig.add_subplot(gs[0, 1])
        self.ax_v = self.fig.add_subplot(gs[1, 1])
        self.ax_q = self.fig.add_subplot(gs[2, 1])

        self.ax_p.set_title("Position vs Time")
        self.ax_v.set_title("Velocity vs Time")
        self.ax_q.set_title("Quaternion vs Time")

        self.ax_p.set_ylabel("p [m]")
        self.ax_v.set_ylabel("v [m/s]")
        self.ax_q.set_ylabel("q [-]")
        self.ax_q.set_xlabel("Time [s]")

        # curves
        self.p_lines = self._make_lines(self.ax_p, 3)
        self.v_lines = self._make_lines(self.ax_v, 3)
        self.q_lines = self._make_lines(self.ax_q, 4)

        plt.ion()

        # subscribe
        self.stage.subscribe(SampleType.STATE, lambda s: self.buffer.append(s))

    # Create n colored lines in an axis
    @staticmethod
    def _make_lines(ax, n):
        colors = ["r", "g", "b", "k"]
        lines = []
        for i in range(n):
            (l,) = ax.plot([], [], colors[i % len(colors)])
            lines.append(l)
        return lines

    @staticmethod
    def base_triangle():
        return np.array([[0.35, 0.0], [-0.2, 0.15], [-0.2, -0.15]])

    @staticmethod
    def extract_yaw(quaternion):
        v = quaternion.apply([1, 0, 0])
        return np.arctan2(v[1], v[0])

    def update_plot(self, frame):
        if not self.buffer:
            return []

        states = list(self.buffer)

        # -------- PATH --------
        xs = [s.p[0] for s in states]
        ys = [s.p[1] for s in states]
        self.path_line.set_data(xs, ys)

        # -------- HEADING ARROWS --------
        for arr in self.heading_history:
            arr.remove()
        self.heading_history.clear()

        for s in states[:-1]:
            p = s.p[:2]
            yaw = self.extract_yaw(s.q)
            dx, dy = 0.3 * np.cos(yaw), 0.3 * np.sin(yaw)
            arrow = self.ax_pose.arrow(
                p[0],
                p[1],
                dx,
                dy,
                head_width=0.08,
                head_length=0.12,
                color="0.7",
                alpha=0.6,
                length_includes_head=True,
            )
            self.heading_history.append(arrow)

        # -------- CURRENT TRIANGLE --------
        latest = states[-1]
        p = latest.p[:2]
        yaw = self.extract_yaw(latest.q)
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        tri = (R @ self.base_triangle().T).T + p
        self.triangle.set_xy(tri)

        # center view
        r = 50
        self.ax_pose.set_xlim(p[0] - r, p[0] + r)
        self.ax_pose.set_ylim(p[1] - r, p[1] + r)

        # -------- TIME SERIES --------
        times = [s.timestamp * 1e-9 for s in states]

        # p(t)
        for i in range(3):  # x,y,z
            self.p_lines[i].set_data(times, [s.p[i] for s in states])

        # v(t)
        for i in range(3):
            self.v_lines[i].set_data(times, [s.v[i] for s in states])

        # q(t)
        for i in range(4):  # qw, qx, qy, qz
            self.q_lines[i].set_data(times, [s.q.as_quat()[i] for s in states])

        # autoscale
        for ax in (self.ax_p, self.ax_v, self.ax_q):
            ax.relim()
            ax.autoscale_view()

        return (
            [self.path_line, self.triangle]
            + self.heading_history
            + self.p_lines
            + self.v_lines
            + self.q_lines
        )

    def start(self, interval=50):
        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=interval,
            blit=False,
            cache_frame_data=False,
        )
        plt.show(block=True)
