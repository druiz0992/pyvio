import unittest
from scipy.spatial.transform import Rotation as R
from collections import deque

from pyvio.core.domain.sample import SensorSample
from pyvio.core.ports.sample import SampleType
from pyvio.core.services.interpolator import (
    LinearInterpolator,
    MidpointInterpolator,
    find_bracket,
)


class TestInterpolator(unittest.TestCase):
    def setUp(self):
        # Simple buffer with two sensor samples
        self.s0 = SensorSample(SampleType.ACCELEROMETER, 0, 0.0, 0.0, 0.0)
        self.s1 = SensorSample(SampleType.ACCELEROMETER, 10, 10.0, 20.0, 30.0)
        self.acc_buffer = deque([self.s0, self.s1], maxlen=10)

        self.g0 = SensorSample(SampleType.GYROSCOPE, 0, 0.0, 0.0, 0.0)
        self.g1 = SensorSample(SampleType.GYROSCOPE, 10, 10.0, 20.0, 30.0)
        self.gyro_buffer = deque([self.g0, self.g1], maxlen=10)

    def test_find_bracket(self):
        res = find_bracket(self.acc_buffer, 5)
        self.assertIsNotNone(res)

        # Or more explicit:
        if res is not None:
            s0, s1, alpha = res
        else:
            self.fail("find_bracket returned None")

        self.assertEqual(s0.timestamp, 0)
        self.assertEqual(s1.timestamp, 10)
        self.assertAlmostEqual(alpha, 0.5)

    def test_linear_interpolation(self):
        interp = LinearInterpolator(self.acc_buffer, self.gyro_buffer)
        result = interp.interpolate(self.acc_buffer, 5)
        self.assertIsNotNone(result)

        if result is not None:
            self.assertAlmostEqual(result.x, 5.0)
            self.assertAlmostEqual(result.y, 10.0)
            self.assertAlmostEqual(result.z, 15.0)

    def test_midpoint_interpolation(self):
        interp = MidpointInterpolator(self.acc_buffer, self.gyro_buffer)
        result = interp.interpolate(self.acc_buffer, 5)
        # Midpoint: v_mid = (v0 + v1)/2 = [5,10,15], then v = v0*(1-alpha)+v_mid*alpha
        # alpha = 0.5, v0 = [0,0,0], v = 0.5*v_mid = [2.5,5,7.5]
        self.assertIsNotNone(result)

        if result is not None:
            self.assertAlmostEqual(result.x, 2.5)
            self.assertAlmostEqual(result.y, 5.0)
            self.assertAlmostEqual(result.z, 7.5)
