import numpy as np
import time
from collections import deque

from pyvio.core.domain.sample import SensorSample
from pyvio.core.ports.sample import SampleType

class Simulator:
    def __init__(
        self,
        dt_imu: float = 0.01,
        dt_gps: float = 1.0,
        accel_bias: np.ndarray | None = None,
        gyro_bias: np.ndarray | None = None,
        accel_noise_std: float = 0.02,
        gyro_noise_std: float = 0.001,
        gps_noise_std: float = 1.0,
        velocity_noise_std: float = 0.05,
    ):
        # IMU/GPS rates
        self.dt_imu = dt_imu
        self.dt_gps = dt_gps

        # Noise and bias
        self.accel_bias = accel_bias if accel_bias is not None else np.zeros(3)
        self.gyro_bias = gyro_bias if gyro_bias is not None else np.zeros(3)
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std
        self.gps_noise_std = gps_noise_std
        self.velocity_noise_std = velocity_noise_std

        # Gravity
        self.gravity = np.array([0, 0, 9.81])

        # State
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = 0.0  # yaw in radians
        self.imu_time = 0.0
        self.gps_time = 0.0

        # Buffers
        self.imu_buffer = deque()
        self.gps_buffer = deque()

    def step_imu(self):
        """Generate one IMU sample (accelerometer + gyro)"""
        omega = 0.1  # rad/s rotation rate in yaw
        forward_accel = 0.5  # m/s^2

        # Update orientation
        self.orientation += omega * self.dt_imu

        # Compute acceleration in world frame
        acc_world = np.array([forward_accel, 0, 0])
        # Rotate to body frame (2D yaw only)
        c, s = np.cos(self.orientation), np.sin(self.orientation)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        acc_body = R.T @ (acc_world + self.gravity)

        # Add noise and bias
        acc_sample = acc_body + self.accel_bias + np.random.randn(3) * self.accel_noise_std
        gyro_sample = np.array([0, 0, omega]) + self.gyro_bias + np.random.randn(3) * self.gyro_noise_std

        # Update velocity and position
        self.velocity += acc_world * self.dt_imu
        self.position += self.velocity * self.dt_imu

        # Create SensorSample instances
        accel_s = SensorSample(SampleType.ACCELEROMETER, int(self.imu_time*1e9), *acc_sample)
        gyro_s = SensorSample(SampleType.GYROSCOPE, int(self.imu_time*1e9), *gyro_sample)

        self.imu_buffer.append(accel_s)
        self.imu_buffer.append(gyro_s)

        self.imu_time += self.dt_imu
        print(f"Accel: {accel_s}")
        print(f"Gyro: {gyro_s}")

    def step_gps(self):
        """Generate GPS + velocity sample"""
        gps_pos = self.position + np.random.randn(3) * self.gps_noise_std

        vel_body = np.array([self.velocity[0], 0, 0])
        c, s = np.cos(self.orientation), np.sin(self.orientation)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        vel_global = R @ vel_body + np.random.randn(3) * self.velocity_noise_std

        gps_s = SensorSample(SampleType.GPS, int(self.gps_time*1e9), *gps_pos)
        vel_s = SensorSample(SampleType.ODOMETRY, int(self.gps_time*1e9), *vel_global)

        self.gps_buffer.append(gps_s)
        self.gps_buffer.append(vel_s)

        print(f"GPS: {gps_s}")
        print(f"Vel: {vel_s}")
        self.gps_time += self.dt_gps

    def run(self, duration: float):
        """Run simulator for a given duration in seconds"""
        t = 0.0
        while t < duration:
            self.step_imu()
            if self.imu_time >= self.gps_time:
                self.step_gps()
            t += self.dt_imu
            time.sleep(self.dt_imu)  # remove for faster-than-real-time simulation


# -----------------------------
# Example usage
# -----------------------------
if __name__ == "__main__":
    sim = Simulator(dt_imu=0.01, dt_gps=1.0)
    sim.run(duration=5.0)

    print("Generated IMU samples:", len(sim.imu_buffer))
    print("Generated GPS/ODO samples:", len(sim.gps_buffer))

