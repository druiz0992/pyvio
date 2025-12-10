import unittest
import numpy as np
from pyvio.core.services.eskf import ESKF
from pyvio.core.services.integrator import EulerIntegrator, RK4Integrator

class TestESKF(unittest.TestCase):

    def setUp(self):
        # small Q for stable tests
        Q = np.eye(18) * 1e-6
        R = np.eye(3) * 1e-2
        self.dt = 0.01  # 100 Hz

        self.eskf_euler = ESKF(integrator_type=EulerIntegrator, Q=Q, R=R)
        self.eskf_rk4 = ESKF(integrator_type=RK4Integrator, Q=Q, R=R)

    def _predict_steps(self, eskf, w_meas, a_meas, steps):
        """Helper: do multiple prediction steps."""
        timestamp = 0
        for _ in range(steps):
            timestamp += int(self.dt * 1e9)  # nanoseconds
            eskf.predict(w_meas, a_meas, timestamp)

    def test_prediction_no_motion(self):
        """Prediction with zero IMU should keep state near initial."""
        for eskf in [self.eskf_euler, self.eskf_rk4]:
            x0 = eskf.x_nom.copy()
            self._predict_steps(eskf, np.zeros(3), np.array([0,0,9.81]), 10)
            np.testing.assert_allclose(eskf.x_nom.v, x0.v, atol=1e-10)
            np.testing.assert_allclose(eskf.x_nom.p, x0.p, atol=1e-10)

    def test_gps_update_after_motion(self):
        """GPS update should correct position after multiple predictions."""
        for eskf in [self.eskf_euler, self.eskf_rk4]:
            w_meas = np.zeros(3)
            a_meas = np.array([0, 0, 9.81])
            self._predict_steps(eskf, w_meas, a_meas, 50)  # half-second motion

            # GPS measurement
            gps_meas = np.array([0.5, 0.0, -2.0])
            eskf.update_position(gps_meas)

            # Position should be corrected
            np.testing.assert_allclose(eskf.x_nom.p, gps_meas, atol=1e-1)

    def test_velocity_update_after_motion(self):
        """Velocity update should correct after multiple predictions."""
        for eskf in [self.eskf_euler, self.eskf_rk4]:
            w_meas = np.zeros(3)
            a_meas = np.array([0, 0, 9.81])
            self._predict_steps(eskf, w_meas, a_meas, 50)

            vel_meas = np.array([0.1, 0.2, -0.1])
            eskf.update_velocity(vel_meas)
            np.testing.assert_allclose(eskf.x_nom.v, vel_meas, atol=1e-2)

    def test_multiple_predictions_before_update(self):
        """Check ESKF accumulates predictions before measurement update."""
        for eskf in [self.eskf_euler, self.eskf_rk4]:
            w_meas = np.array([0.01, 0.0, 0.0])
            a_meas = np.array([0.0, 0.0, 9.81])

            self._predict_steps(eskf, w_meas, a_meas, 100)
            # Should have nonzero motion
            self.assertFalse(np.allclose(eskf.x_nom.p, 0))
            self.assertFalse(np.allclose(eskf.x_nom.v, 0))

            gps_meas = np.array([1.0, 0.0, -1.0])
            eskf.update_position(gps_meas)
            np.testing.assert_allclose(eskf.x_nom.p, gps_meas, atol=1e-2)


if __name__ == "__main__":
    unittest.main()
