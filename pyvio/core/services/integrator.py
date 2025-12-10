import numpy as np
from scipy.spatial.transform import Rotation as R

from pyvio.core.domain.state import State
from pyvio.core.ports.integrator import StateIntegrator


class EulerIntegrator(StateIntegrator):
    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:
        new_state = state.copy()
        deriv = state.dynamics(w, a)
        
        new_state.p = state.p + state.v * dt + 0.5 * deriv["v"] * dt**2
        new_state.v = state.v + dt * deriv["v"]
        
        dq_vec = deriv["q"] * dt
        if np.linalg.norm(dq_vec) > 0:
            dq = R.from_rotvec(dq_vec)
        else:
            dq = R.identity()

        new_state.q = state.q * dq
        new_state.normalize_orientation()

        new_state.timestamp = int(state.timestamp + dt * 1e9)
        return new_state

class MidpointIntegrator(StateIntegrator):
    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:
        deriv1 = state.dynamics(w, a)

        v_mid = state.v + 0.5 * deriv1["v"] * dt

        dq_mid_vec = 0.5 * deriv1["q"] * dt
        if np.linalg.norm(dq_mid_vec) > 0:
            q_mid = state.q * R.from_rotvec(dq_mid_vec)
        else:
            q_mid = state.q

        mid_state = state.copy()
        mid_state.v = v_mid
        mid_state.q = q_mid
        deriv2 = mid_state.dynamics(w, a)

        new_state = state.copy()
        new_state.p = state.p + v_mid * dt
        new_state.v = state.v + deriv2["v"] * dt
        
        dq_vec = deriv2["q"] * dt
        if np.linalg.norm(dq_vec) > 0:
            dq = R.from_rotvec(dq_vec)
        else:
            dq = R.identity()
        
        new_state.q = state.q * dq
        new_state.normalize_orientation()
        new_state.timestamp = int(state.timestamp + dt * 1e9)
        return new_state


class RK4Integrator(StateIntegrator):
    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:
        # k1
        k1 = state.dynamics(w, a)

        # k2
        s2 = state.copy()
        s2.p += 0.5 * state.v * dt + 0.125 * k1["v"] * dt**2
        s2.v += 0.5 * k1["v"] * dt
        dq_vec = 0.5 * k1["q"] * dt
        s2.q = state.q * (R.from_rotvec(dq_vec) if np.linalg.norm(dq_vec) > 0 else R.identity())
        s2.normalize_orientation()
        k2 = s2.dynamics(w, a)

        # k3
        s3 = state.copy()
        s3.p += 0.5 * state.v * dt + 0.125 * k2["v"] * dt**2
        s3.v += 0.5 * k2["v"] * dt
        dq_vec = 0.5 * k2["q"] * dt
        s3.q = state.q * (R.from_rotvec(dq_vec) if np.linalg.norm(dq_vec) > 0 else R.identity())
        s3.normalize_orientation()
        k3 = s3.dynamics(w, a)

        # k4
        s4 = state.copy()
        s4.p += state.v * dt + 0.5 * k3["v"] * dt**2
        s4.v += k3["v"] * dt
        dq_vec = k3["q"] * dt
        s4.q = state.q * (R.from_rotvec(dq_vec) if np.linalg.norm(dq_vec) > 0 else R.identity())
        s4.normalize_orientation()
        k4 = s4.dynamics(w, a)

        # combine
        new_state = state.copy()
        new_state.p += state.v * dt + dt**2 / 6 * (k1["v"] + 2*k2["v"] + 2*k3["v"] + k4["v"])
        new_state.v += dt / 6 * (k1["v"] + 2*k2["v"] + 2*k3["v"] + k4["v"])

        dq_vec = dt / 6 * (k1["q"] + 2*k2["q"] + 2*k3["q"] + k4["q"])
        dq = R.from_rotvec(dq_vec) if np.linalg.norm(dq_vec) > 0 else R.identity()
        new_state.q = state.q * dq

        new_state.normalize_orientation()
        new_state.timestamp = int(state.timestamp + dt * 1e9)
        return new_state
