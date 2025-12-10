import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

from pyvio.core.domain.state import State
from pyvio.core.domain.integrator import (
    EulerIntegrator,
    MidpointIntegrator,
    RK4Integrator,
)


class TestIntegrator(unittest.TestCase):
    def setUp(self):
        self.state = State.default()
        self.dt = 1.0
        self.w = np.array([0.0, 0.0, 0.0])
        self.a = np.array([0.0, 0.0, 0.0])

    def test_euler_integrator(self):
        integrator = EulerIntegrator()
        new_state = integrator.integrate(self.state, self.w, self.a, self.dt)
        np.testing.assert_allclose(new_state.p, self.state.p + self.state.v * self.dt)
        np.testing.assert_allclose(new_state.v, self.state.v + self.state.g * self.dt)

    def test_midpoint_integrator(self):
        integrator = MidpointIntegrator()
        new_state = integrator.integrate(self.state, self.w, self.a, self.dt)
        np.testing.assert_allclose(new_state.p, self.state.p + self.state.v * self.dt)
        np.testing.assert_allclose(new_state.v, self.state.v + self.state.g * self.dt)

    def test_rk4_integrator(self):
        integrator = RK4Integrator()
        new_state = integrator.integrate(self.state, self.w, self.a, self.dt)
        # For zero w and a, RK4 should integrate gravity into velocity
        np.testing.assert_allclose(new_state.v, self.state.v + self.state.g * self.dt)
        np.testing.assert_allclose(
            new_state.p,
            self.state.p + self.state.v * self.dt + 0.5 * self.state.g * self.dt**2,
            rtol=1e-6,
        )


if __name__ == "__main__":
    unittest.main()
