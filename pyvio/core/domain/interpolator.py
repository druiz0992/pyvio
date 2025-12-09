import bisect
from collections import deque
from typing import Optional, Tuple, List

from pyvio.core.domain.sample import SensorSample
from pyvio.core.ports.sample import SampleType
from pyvio.core.ports.interpolator import IMUInterpolator


class LinearInterpolator(IMUInterpolator):
    """Simple linear interpolation between two IMU samples."""
    
    def interpolate(self, buffer, t_query) -> Optional[SensorSample]:
        bracket = find_bracket(buffer, t_query)
        
        if bracket is None:
            return None
        
        s0, s1, alpha = bracket
        v0 = s0.as_array().astype(float)
        v1 = s1.as_array().astype(float)
        
        v = (1.0 - alpha) * v0 + alpha * v1
        return SensorSample(SampleType.IMU, t_query, v[0], v[1], v[2])
        


class MidpointInterpolator(IMUInterpolator):
    """Midpoint / RK2-style interpolation for IMU."""
    
    def interpolate(self, buffer, t_query) -> Optional[SensorSample]:
        bracket = find_bracket(buffer, t_query)
        
        if bracket is None:
            return None
        
        s0, s1, alpha = bracket
        v0 = s0.as_array().astype(float)
        v1 = s1.as_array().astype(float)

        v_mid = 0.5 * (v0 + v1)
        v = v0 * (1.0 - alpha) + v_mid * alpha
        
        return SensorSample(SampleType.IMU, t_query, v[0], v[1], v[2])




def find_bracket(buffer: deque, t_query: int) -> Optional[Tuple[SensorSample, SensorSample, float]]:
    """
    Returns (s0, s1, α) where:
        s0.timestamp <= t_query <= s1.timestamp
        α is interpolation weight in [0,1]

    Returns None if cannot interpolate.
    """

    if len(buffer) < 2:
        return None

    samples: List = list(buffer)
    times = [s.timestamp for s in buffer]

    if t_query < times[0] or t_query > times[-1]:
        return None

    idx = bisect.bisect_left(times, t_query)

    if times[idx] == t_query:
        return samples[idx], samples[idx], 0.0
    
    s0, s1 = samples[idx - 1], samples[idx]
    t0, t1 = s0.timestamp, s1.timestamp
    
    alpha = (t_query - t0) / (t1 - t0)

    return s0, s1, float(alpha)
        