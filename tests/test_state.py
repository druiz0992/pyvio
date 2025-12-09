import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

from pyvio.core.domain.state import State


class TestState(unittest.TestCase):
    def test_copy_and_array(self):
        state = State.default()
        copy = state.copy()
        arr = state.as_array()
        self.assertEqual(len(arr), 20)
        self.assertTrue(np.allclose(arr[1:4], state.p))
        self.assertTrue(np.allclose(copy.p, state.p))
        self.assertIsNot(copy.p, state.p)

    def test_from_vector(self):
        state = State.default()
        arr = state.as_array()
        new_state = State.from_vector(arr)
        self.assertTrue(np.allclose(new_state.p, state.p))
        self.assertTrue(np.allclose(new_state.v, state.v))
        self.assertTrue(np.allclose(new_state.bg, state.bg))

    def test_bytes_roundtrip(self):
        state = State.default()
        b = state.to_bytes()
        new_state = State.from_bytes(b)
        self.assertEqual(state.timestamp, new_state.timestamp)
        self.assertTrue(np.allclose(state.p, new_state.p))
        self.assertTrue(np.allclose(state.q.as_quat(), new_state.q.as_quat()))

    def test_dynamics(self):
        state = State.default()
        w = np.array([0.1, 0.2, 0.3])
        a = np.array([0.5, -0.2, 9.7])
        deriv = state.dynamics(w, a)
        self.assertEqual(set(deriv.keys()), {"p", "v", "q"})
        self.assertEqual(len(deriv["p"]), 3)
        self.assertEqual(len(deriv["v"]), 3)
        self.assertEqual(len(deriv["q"]), 3)


if __name__ == "__main__":
    unittest.main()
