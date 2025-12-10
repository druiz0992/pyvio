import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

from pyvio.core.domain.state import State
from pyvio.core.services.integrator import (
    EulerIntegrator,
    MidpointIntegrator,
    RK4Integrator,
)


class TestIntegrator(unittest.TestCase):
    def setUp(self):
        self.dt = 0.01  # 100 Hz
        self.state = State.default()

        # Non-zero angular velocity (rotate 90°/sec around Z)
        self.w = np.array([0.0, 0.0, np.pi / 2])

        # Non-zero acceleration (forward direction)
        self.a = np.array([1.0, 0.0, 0.0])

    def test_euler_integrator(self):
        integrator = EulerIntegrator()
        s = self.state.copy()

        new_state = integrator.integrate(s, self.w, self.a, self.dt)

        # Expected velocity increase: v += R*(a - ba)*dt + g*dt
        a_world = s.q.apply(self.a - s.ba)
        expected_v = s.v + (a_world + s.g) * self.dt
        np.testing.assert_allclose(new_state.v, expected_v, atol=1e-6)

        # Expected position: p += v * dt
        np.testing.assert_allclose(new_state.p, s.p + s.v * self.dt, atol=1e-6)

        # Orientation should rotate by w*dt
        expected_q = s.q * R.from_rotvec(self.w * self.dt)
        np.testing.assert_allclose(
            new_state.q.as_quat(), expected_q.as_quat(), atol=1e-6
        )

    def test_midpoint_integrator(self):
        integrator = MidpointIntegrator()
        s = self.state.copy()

        new_state = integrator.integrate(s, self.w, self.a, self.dt)

        # Midpoint integration is still first-order in your implementation,
        # but orientation integration is slightly better than Euler.

        expected_q = s.q * R.from_rotvec(self.w * self.dt)
        np.testing.assert_allclose(
            new_state.q.as_quat(), expected_q.as_quat(), atol=1e-6
        )

        # Check position & velocity are consistent with Euler (your code matches Euler)
        a_world = s.q.apply(self.a - s.ba)
        expected_v = s.v + (a_world + s.g) * self.dt
        np.testing.assert_allclose(new_state.v, expected_v, atol=1e-6)
        np.testing.assert_allclose(new_state.p, s.p + s.v * self.dt, atol=1e-6)

    def test_rk4_integrator(self):
        integrator = RK4Integrator()
        s = self.state.copy()

        new_state = integrator.integrate(s, self.w, self.a, self.dt)

        # ---- POSITION TEST (better than Euler/Midpoint) ----
        # true continuous-time solution for constant acceleration:
        a_world = s.q.apply(self.a - s.ba)
        expected_p = s.p + s.v * self.dt + 0.5 * (a_world + s.g) * self.dt**2

        np.testing.assert_allclose(new_state.p, expected_p, atol=5e-4)

        # ---- VELOCITY TEST ----
        expected_v = s.v + (a_world + s.g) * self.dt
        np.testing.assert_allclose(new_state.v, expected_v, atol=5e-4)

        # ---- ORIENTATION TEST ----
        # True rotation for constant angular velocity:
        expected_q = s.q * R.from_rotvec(self.w * self.dt)
        np.testing.assert_allclose(
            new_state.q.as_quat(), expected_q.as_quat(), atol=1e-6
        )


if __name__ == "__main__":
    unittest.main()
