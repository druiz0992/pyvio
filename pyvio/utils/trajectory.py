import numpy as np

from pyvio.core.ports.trajectory import TrajectoryPort



class CircleTrajectory(TrajectoryPort):
    def evaluate(self, t: float):
        r, v = self.radius, self.speed
        x = r * np.cos(v * t / r)
        y = r * np.sin(v * t / r)
        z = 0.0
        dx = -v * np.sin(v * t / r)
        dy = v * np.cos(v * t / r)
        dz = 0.0
        yaw = np.arctan2(dy, dx)
        return np.array([x, y, z]), np.array([dx, dy, dz]), yaw


class Figure8Trajectory(TrajectoryPort):
    def evaluate(self, t: float):
        r, v = self.radius, self.speed
        x = r * np.sin(v * t / r)
        y = r * np.sin(v * t / r) * np.cos(v * t / r)
        z = 0.0
        dx = v * np.cos(v * t / r)
        dy = v * (np.cos(v * t / r) ** 2 - np.sin(v * t / r) ** 2)
        dz = 0.0
        yaw = np.arctan2(dy, dx)
        return np.array([x, y, z]), np.array([dx, dy, dz]), yaw


class SpiralTrajectory(TrajectoryPort):
    def __init__(self, radius=5.0, height=5.0, speed=1.0, period=1.0):
        super().__init__(radius, height, speed)
        self.period = period
        
    def evaluate(self, t: float):
        r, v, h, T = self.radius, self.speed, self.height, self.period

        direction_up = int(t / T) % 2 == 0  # even -> up, odd -> down

        x = r * np.cos(v * t / r)
        y = r * np.sin(v * t / r)

        segment_t = t % T  # time within current up/down segment
        dz = h * (segment_t / T) if direction_up else h * (1 - segment_t / T)
        z = dz if direction_up else dz

        dx = -v * np.sin(v * t / r)
        dy = v * np.cos(v * t / r)
        dz_vel = h / T if direction_up else -h / T

        yaw = np.arctan2(dy, dx)

        return np.array([x, y, z]), np.array([dx, dy, dz_vel]), yaw



class Figure8ZTrajectory(TrajectoryPort):
    def __init__(self, radius=5.0, height=5.0, speed=1.0, period: float = 1.0):
        super().__init__(radius, height, speed)
        self.period = period  # duration of each up/down segment

    def evaluate(self, t: float):
        r, v, h, T = self.radius, self.speed, self.height, self.period

        # Determine direction based on integer part of t / period
        direction_up = int(t / T) % 2 == 0  # even -> up, odd -> down

        # x, y for figure-8
        x = r * np.sin(v * t / r)
        y = r * np.sin(v * t / r) * np.cos(v * t / r)

        # z position: linearly go up or down within segment
        segment_t = t % T
        z = h * (segment_t / T) if direction_up else h * (1 - segment_t / T)

        # Velocities
        dx = v * np.cos(v * t / r)
        dy = v * (np.cos(v * t / r) ** 2 - np.sin(v * t / r) ** 2)
        dz = h / T if direction_up else -h / T

        yaw = np.arctan2(dy, dx)

        return np.array([x, y, z]), np.array([dx, dy, dz]), yaw
