import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from typing import cast
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3D

from pyvio.core.ports.sample import SampleType
from .stage import Stage


class LiveVisualizerState3D:
    def __init__(self, stage: Stage, maxlen=2000):
        self.stage = stage
        self.buffer = deque(maxlen=maxlen)

        # --- Figure with grid layout ---
        self.fig = plt.figure(figsize=(16, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[2, 1])

        # --- Left: 3D POSE PLOT ---
        self.ax_pose = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax_pose.set_xlabel("X [m]")
        self.ax_pose.set_ylabel("Y [m]")
        self.ax_pose.set_zlabel("Z [m]")

        # Trajectory line (type hint to silence warnings)
        self.path_line: Line3D = cast(
            Line3D, self.ax_pose.plot([], [], [], color="0.6", linewidth=1)[0]
        )

        # Heading arrow (updated per frame)
        self.heading_arrow = None

        # Robot body (simple 3D triangle pyramid)
        self.body_lines = []
        self.base_body = self._build_body()

        # --- Right: time-series plots ---
        self.ax_p = self.fig.add_subplot(gs[0, 1])
        self.ax_v = self.fig.add_subplot(gs[1, 1])
        self.ax_q = self.fig.add_subplot(gs[2, 1])

        self.ax_p.set_title("Position vs Time")
        self.ax_v.set_title("Velocity vs Time")
        self.ax_q.set_title("Quaternion vs Time")

        self.ax_p.set_ylabel("p [m]")
        self.ax_v.set_ylabel("v [m/s]")
        self.ax_q.set_ylabel("q [-]")
        self.ax_q.set_xlabel("t [s]")

        # Timeseries lines
        self.p_lines = self._make_lines(self.ax_p, 3)
        self.v_lines = self._make_lines(self.ax_v, 3)
        self.q_lines = self._make_lines(self.ax_q, 4)

        plt.ion()
        self.stage.subscribe(SampleType.STATE, lambda s: self.buffer.append(s))

    @staticmethod
    def _make_lines(ax, n):
        colors = ["r", "g", "b", "k"]
        lines = []
        for i in range(n):
            (l,) = ax.plot([], [], colors[i % len(colors)])
            lines.append(l)
        return lines

    def _build_body(self):
        return np.array(
            [
                [0.4, 0, 0],  # nose
                [-0.2, 0.15, 0.15],
                [-0.2, -0.15, 0.15],
                [-0.2, 0, -0.2],
            ]
        )

    @staticmethod
    def extract_yaw(quaternion):
        v = quaternion.apply([1, 0, 0])
        return np.arctan2(v[1], v[0])

    @staticmethod
    def transform(points, q, p):
        return np.array([q.apply(pt) + p for pt in points])

    def update_plot(self, frame):
        if not self.buffer:
            return []

        states = list(self.buffer)

        # -------- PATH --------
        xs = [s.p[0] for s in states]
        ys = [s.p[1] for s in states]
        zs = [s.p[2] for s in states]

        self.path_line.set_data(xs, ys)
        self.path_line.set_3d_properties(zs)  # type: ignore[arg-type]

        # -------- CURRENT POSE --------
        latest = states[-1]
        p = latest.p
        q = latest.q

        # Remove old body
        for bl in self.body_lines:
            bl.remove()
        self.body_lines.clear()

        # Draw body edges
        body_pts = self.transform(self.base_body, q, p)
        for i in range(1, 4):
            (line,) = self.ax_pose.plot(
                [body_pts[0, 0], body_pts[i, 0]],
                [body_pts[0, 1], body_pts[i, 1]],
                [body_pts[0, 2], body_pts[i, 2]],
                color="blue",
            )
            self.body_lines.append(line)

        # -------- HEADING ARROW --------
        if self.heading_arrow:
            self.heading_arrow.remove()

        yaw = self.extract_yaw(q)
        dx, dy, dz = np.cos(yaw), np.sin(yaw), 0.0

        self.heading_arrow = self.ax_pose.quiver(
            p[0],
            p[1],
            p[2],
            dx,
            dy,
            dz,
            length=1.0,  # type: ignore[arg-type]
            normalize=False,
            color="green",
        )

        # -------- CAMERA LIMITS --------
        r = 50
        self.ax_pose.set_xlim(p[0] - r, p[0] + r)
        self.ax_pose.set_ylim(p[1] - r, p[1] + r)
        self.ax_pose.set_zlim(p[2] - r, p[2] + r)

        # -------- TIME SERIES --------
        times = [s.timestamp * 1e-9 for s in states]

        for i in range(3):
            self.p_lines[i].set_data(times, [s.p[i] for s in states])
            self.v_lines[i].set_data(times, [s.v[i] for s in states])

        for i in range(4):
            self.q_lines[i].set_data(times, [s.q.as_quat()[i] for s in states])

        # autoscale
        for ax in (self.ax_p, self.ax_v, self.ax_q):
            ax.relim()
            ax.autoscale_view()

        return (
            [self.path_line, self.heading_arrow]
            + self.body_lines
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
