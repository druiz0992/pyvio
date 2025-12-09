import numpy as np
from scipy.spatial.transform import Rotation as R

from pyvio.core.domain.state import State
from pyvio.core.ports.integrator import StateIntegrator


class EulerIntegrator(StateIntegrator):
    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:
        new_state = state.copy()
        deriv = state.dynamics(w, a)
        new_state.p = state.p + dt * deriv['p']
        new_state.v = state.v + dt * deriv['v']
        dq = R.from_rotvec(deriv['q'] * dt)
        new_state.q = state.q * dq
    
        return new_state
  
class RK4Integrator(StateIntegrator):

    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:

        # k1
        k1 = state.dynamics(w, a)

        # k2
        s2 = state.copy()
        s2.p += 0.5 * dt * k1['p']
        s2.v += 0.5 * dt * k1['v']
        s2.q = (state.q * R.from_rotvec(0.5 * dt * k1['q']))
        s2.normalize_orientation()
        k2 = s2.dynamics(w, a)

        # k3
        s3 = state.copy()
        s3.p += 0.5 * dt * k2['p']
        s3.v += 0.5 * dt * k2['v']
        s3.q = (state.q * R.from_rotvec(0.5 * dt * k2['q']))
        s3.normalize_orientation()
        k3 = s3.dynamics(w, a)

        # k4
        s4 = state.copy()
        s4.p += dt * k3['p']
        s4.v += dt * k3['v']
        s4.q = (state.q * R.from_rotvec(dt * k3['q']))
        s4.normalize_orientation()
        k4 = s4.dynamics(w, a)

        # combine
        new_state = state.copy()
        new_state.p += dt * (k1['p'] + 2*k2['p'] + 2*k3['p'] + k4['p']) / 6
        new_state.v += dt * (k1['v'] + 2*k2['v'] + 2*k3['v'] + k4['v']) / 6

        dq = R.from_rotvec(
            dt * (k1['q'] + 2*k2['q'] + 2*k3['q'] + k4['q']) / 6
        )
        new_state.q = (state.q * dq)
        new_state.normalize_orientation()

        new_state.timestamp = int(state.timestamp + dt * 1e9)
        return new_state


class MidpointIntegrator(StateIntegrator):
    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:
        new_state = state.copy()
        deriv = state.dynamics(w, a)
        
        new_state.p = state.p + dt * deriv['p']
        new_state.v = state.v + dt * deriv['v']

        dq = R.from_rotvec(deriv['q'] * dt)
        new_state.q = (state.q * dq)
        new_state.normalize_orientation()

        new_state.timestamp = int(state.timestamp + dt * 1e9)
        
        return new_state