import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation
from ahrs.filters import Madgwick
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from pyvio.core.domain.samples import SensorType, SensorSample
from .stage import Stage


class Ahrs:
    def __init__(self, stage: Stage, maxlen=100):
        self.stage = stage
        self.buffers = {s: deque(maxlen=maxlen) for s in SensorType.imu_list()}
        self.madgwick = Madgwick()
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # Last sensor samples
        self.last_sample = [
            SensorSample(SensorType.ACCELEROMETER, 0, 0, 0, 0),
            SensorSample(SensorType.GYROSCOPE, 0, 0, 0, 0),
            SensorSample(SensorType.MAGNETOMETER, 0, 0, 0, 0),
        ]

        # Cube geometry
        self.cube_vertices = np.array(
            [
                [-0.5, -0.5, -0.5],
                [0.5, -0.5, -0.5],
                [0.5, 0.5, -0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, 0.5],
                [-0.5, 0.5, 0.5],
            ]
        )
        self.cube_faces = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [1, 2, 6, 5],
            [0, 3, 7, 4],
        ]

        # Setup figure
        self.fig = plt.figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(1, 1, 1, projection="3d")
        self.ax.set_box_aspect([1, 1, 1])
        plt.ion()

        # Subscribe to sensors
        for s in SensorType.imu_list():
            self.stage.subscribe(s, lambda sample, s=s: self.buffers[s].append(sample))

    def quat_to_rotmat(self, q):
        w, x, y, z = q
        return np.array(
            [
                [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
            ]
        )

    def update_filter(self, frame=None):
        for i, sensor in enumerate(SensorType.imu_list()):
            try:
                self.last_sample[i] = self.buffers[sensor].popleft()
            except IndexError:
                pass

        gyro = np.array(self.last_sample[1].as_array())
        accel = np.array(self.last_sample[0].as_array())
        mag = np.array(self.last_sample[2].as_array())

        # Update quaternion
        self.q = self.madgwick.updateMARG(
            q=self.q, gyr=gyro, acc=accel, mag=mag, dt=1 / 400.0
        )

        # Rotate cube
        rot = self.quat_to_rotmat(self.q)
        verts = (rot @ self.cube_vertices.T).T

        # Clear old cube
        for coll in self.ax.collections[:]:
            coll.remove()

        # Draw new cube
        face_vertices = [verts[face] for face in self.cube_faces]
        poly3d = Poly3DCollection(
            face_vertices, facecolors="cyan", alpha=0.4, edgecolors="k"
        )
        self.ax.add_collection3d(poly3d)

        lim = 1
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(-lim, lim)

        return [poly3d]

    def start_animation(self, interval=50):
        self.ani = FuncAnimation(
            self.fig, self.update_filter, frames=100, interval=interval, blit=False
        )
        plt.show(block=True)
