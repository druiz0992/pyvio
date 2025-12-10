from typing import Optional
from scipy.spatial.transform import Rotation as R
import numpy as np
from collections import deque

from pyvio.core.domain.state import State
from pyvio.core.ports.integrator import StateIntegrator
from pyvio.core.services.integrator import EulerIntegrator
from pyvio.utils.lla2enu import LLA2ENU

DIMS = 18

class ESKF:
    def __init__( 
        self,
        state: Optional[State] = None,
        integrator_type: type[StateIntegrator] = EulerIntegrator,
        P: Optional[np.ndarray]=None,
        Q:Optional[np.ndarray]=None,
        R:Optional[np.ndarray]=None
    ):
        
        self.integrator = integrator_type()
        self.x_nom = State.default() if state is None else state
        self.P = np.eye(DIMS) if P is None else P
        self.Q = np.eye(DIMS) * 1e-5 if Q is None else Q
        self.R = np.eye(3) * 1e-2 if R is None else R
        self._last_timestamp: Optional[int] = None
        self._enu_converter: Optional[LLA2ENU] = None

    @staticmethod
    def skew(v: np.ndarray) -> np.ndarray:
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])

    def predict(self, w_meas: np.ndarray, a_meas: np.ndarray, timestamp: int):
        """Propagate nominal state and covariance."""
        # Skip first sample, no dt available
        if self._last_timestamp is None:
            self._last_timestamp = timestamp
            return

        dt = (timestamp - self._last_timestamp) * 1e-9  # convert ns -> seconds

        # Skip too large dt
        if dt <= 0 or dt > 0.1:
            self._last_timestamp = timestamp
            return
        
        self.x_nom = self.integrator.integrate(self.x_nom, w_meas, a_meas, dt)

        # Continuous-time error-state matrix F (18x18)
        F = np.zeros((DIMS, DIMS))
        F[0:3, 3:6] = np.eye(3)              # δṗ = δv
        a_corr = a_meas - self.x_nom.ba
        R_mat = self.x_nom.q.as_matrix()
        F[3:6, 6:9] = -R_mat @ self.skew(a_corr)  # δv̇/δθ
        F[3:6, 12:15] = -R_mat                     # δv̇/δba
        F[3:6, 15:18] = np.eye(3)                  # δv̇/δg

        w_corr = w_meas - self.x_nom.bg
        F[6:9, 6:9] = -self.skew(w_corr)          # δθ̇/δθ
        F[6:9, 9:12] = -np.eye(3)                 # δθ̇/δbg

        # δbġ = 0, δbȧ = 0, δġ = 0 -> already zero

        # Discretize
        Phi = np.eye(DIMS) + F * dt
        self.P = Phi @ self.P @ Phi.T + self.Q * dt
        
        self._last_timestamp = timestamp

    def update_position(self, z: np.ndarray, R: Optional[np.ndarray] = None, lla: bool = False):
        """
        Fuse position measurement.
        If lla=True, z is assumed to be [lat, lon, alt] in degrees/meters.
        """
        # Convert LLA -> ENU if needed
        if lla:
            if self._enu_converter is None:
                # Initialize reference point
                self._enu_converter = LLA2ENU.from_first_sample(*z)
                z_enu = self._enu_converter.convert(*z)
                self.x_nom.p = z_enu.copy()
            else:
                z_enu = self._enu_converter.convert(*z)
        else:
            z_enu = z
            
        print(f"POS {z_enu}")

        # Proceed with normal update
        Rm = self.R if R is None else R
        H = np.zeros((3, DIMS))
        H[:, 0:3] = np.eye(3)  # measure δp
        
        y = z_enu - self.x_nom.p
        
        S = H @ self.P @ H.T + Rm
        
        HPt = (H @ self.P).T
        K = np.linalg.solve(S, HPt.T).T
        
        self._correct_state(K, y, H)

    def update_velocity(self, z: np.ndarray, R: Optional[np.ndarray] = None):
        """Fuse velocity measurement (odometry or GPS velocity)."""
        print(f"VEL {z}")
        Rm = self.R if R is None else R
        
        H = np.zeros((3, DIMS))
        H[:, 3:6] = np.eye(3)  # measure δv
        
        y = z - self.x_nom.v
        
        S = H @ self.P @ H.T + Rm
        HPt = (H @ self.P).T
        K = np.linalg.solve(S, HPt.T).T

        self._correct_state(K, y, H)

    def _correct_state(self, K: np.ndarray, y: np.ndarray, H: np.ndarray):
        """Apply error-state correction."""
        delta_x = K @ y
    
        self.x_nom.p += delta_x[0:3]
        self.x_nom.v += delta_x[3:6]
        dtheta = delta_x[6:9]
        if np.linalg.norm(dtheta) > 1e-12:
            self.x_nom.q = self.x_nom.q * R.from_rotvec(dtheta)
        self.x_nom.bg += delta_x[9:12]
        self.x_nom.ba += delta_x[12:15]
        self.x_nom.g += delta_x[15:18]
        self.x_nom.normalize_orientation()
    
        I = np.eye(DIMS)
        P_new = (I - K @ H) @ self.P  # standard covariance update
        
        # --- Covariance projection (rotation injection correction) ---
        J = np.eye(DIMS)
        J[6:9, 6:9] = np.eye(3) - 0.5 * self.skew(dtheta)
        print(f"{np.linalg.norm(P_new, 'fro')} {np.linalg.norm(J, 'fro')} {dtheta}")

        self.P = J @ P_new @ J.T


class StaticIMUInit:
    def __init__(self, init_time_s=5.0, max_queue_size=500, gravity_norm=9.81, max_gyro_var=5e-1, max_acce_var=1e-2):
        self.init_time_s = init_time_s
        self.max_queue_size = max_queue_size
        self.gravity_norm = gravity_norm
        self.max_gyro_var = max_gyro_var
        self.max_acce_var = max_acce_var

        self.imu_queue: deque = deque(maxlen=max_queue_size)
        self.init_start_time: Optional[int] = None
        self.init_success = False

        self.mean_gyro = np.zeros(3)
        self.mean_acc = np.zeros(3)
        self.gravity = np.array([0, 0, -gravity_norm])

    def add_sample(self, imu):
        """
        imu should have attributes: timestamp (int, ns), gyro (3,), acc (3,)
        """
        if self.init_success:
            return True

        if not self.imu_queue:
            self.init_start_time = imu.timestamp

        self.imu_queue.append(imu)

        # Check if enough time has passed
        elapsed_time = (imu.timestamp - self.init_start_time) * 1e-9  # ns -> seconds
        if elapsed_time >= self.init_time_s:
            self.try_init()
        return self.init_success

    def try_init(self):
        if len(self.imu_queue) < 10:
            return False

        # Stack all gyro and acc measurements
        gyro_stack = np.stack([imu.gyro for imu in self.imu_queue], axis=0)
        acc_stack = np.stack([imu.acc for imu in self.imu_queue], axis=0)

        # Compute mean and covariance (diagonal)
        self.mean_gyro = np.mean(gyro_stack, axis=0)
        cov_gyro = np.var(gyro_stack, axis=0)
        self.mean_acc = np.mean(acc_stack, axis=0)
        cov_acc = np.var(acc_stack, axis=0)

        # Check noise thresholds
        if np.any(cov_gyro > self.max_gyro_var):
            print("Gyro noise too high:", cov_gyro)
            return False
        if np.any(cov_acc > self.max_acce_var):
            print("Accel noise too high:", cov_acc)
            return False

        # Gravity estimation: opposite direction of mean accelerometer
        g_dir = -self.mean_acc / np.linalg.norm(self.mean_acc)
        self.gravity = g_dir * self.gravity_norm

        self.init_success = True
        print("IMU initialized successfully.")
        print("bg =", self.mean_gyro, "ba =", self.mean_acc - self.gravity, "g =", self.gravity)
        return True

    def set_eskf_initial_state(self, eskf):
        """
        Set the initial nominal state of the ESKF after static initialization
        """
        if not self.init_success:
            raise RuntimeError("Static IMU initialization not complete")

        eskf.x_nom.p = np.zeros(3)
        eskf.x_nom.v = np.zeros(3)
        eskf.x_nom.q = R.identity()
        eskf.x_nom.bg = self.mean_gyro
        eskf.x_nom.ba = self.mean_acc - self.gravity
        eskf.x_nom.g = self.gravity

 