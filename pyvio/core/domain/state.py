from __future__ import annotations
import numpy as np
from typing import Optional, Dict
from scipy.spatial.transform import Rotation as R
from enum import Enum

class IntegrationMethod(Enum):
    EULER = 0 
    MIDPOINT = 1
    RK4 = 2
    

G = 9.81
class State:
    def __init__(self, 
                 position: Optional[np.ndarray] = None,
                 velocity: Optional[np.ndarray] = None,
                 orientation: Optional[R] = None,
                 bias_g: Optional[np.ndarray] = None,
                 bias_a: Optional[np.ndarray] = None,
                 g: Optional[np.ndarray] = None,
                 integrator: IntegrationMethod = IntegrationMethod.EULER):
        self.p = np.zeros(3) if position is None else np.array(position)
        self.v = np.zeros(3) if velocity is None else np.array(velocity)
        self.q = R.identity() if orientation is None else orientation
        self.bg = np.zeros(3) if bias_g is None else np.array(bias_g)
        self.ba = np.zeros(3) if bias_a is None else np.array(bias_a)
        self.g = np.array([0, 0, -G]) if g is None else np.array(g)
        self.integrator = integrator

    @classmethod
    def default(cls, integrator: IntegrationMethod = IntegrationMethod.EULER):
        """Return a default zero state"""
        return cls(
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=R.identity(),
            bias_g=np.zeros(3),
            bias_a=np.zeros(3),
            g=np.array([0,0,-G]),
            integrator=integrator
        )
        
    @classmethod
    def from_vector(cls, x: np.ndarray, integrator: IntegrationMethod = IntegrationMethod.EULER):
        p = x[0:3]
        v = x[3:6]
        q = R.from_quat(x[6:10])
        bg = x[10:13]
        ba = x[13:16]
        g = np.array([0,0,-G])
        
        return cls(p, v, q, bg, ba, g, integrator)
    
    def to_vector(self) -> np.ndarray :
        """
        Flatten the state into a 16-element vector:
        [p(3), v(3), q(4), bias_g(3), bias_a(3)]
        """
        p_vec = self.p
        v_vec = self.v
        q_vec = self.q.as_quat()
        bg_vec = self.bg
        ba_vec = self.ba
        g_vec = self.g

        return np.concatenate([p_vec, v_vec, q_vec, bg_vec, ba_vec, g_vec])
    
    def copy(self) -> State:
        """
        Return a full copy of the current state.
        """
        return State(
            position=self.p.copy(),
            velocity=self.v.copy(),
            orientation=self.q,  # Rotation objects are immutable-safe
            bias_g=self.bg.copy(),
            bias_a=self.ba.copy(),
            g=self.g.copy(),
            integrator=self.integrator
        )
    
    def dynamics(self, w_meas:np.ndarray, a_meas:np.ndarray) -> Dict[str, np.ndarray]:
        """
           Compute derivatives of position, velocity, orientation
           
           p_dot = v
           v_dot = R * (a - ba) + g
           R_dot = R * (w - bg)^ => we keep it as a vector w - bg
        """
        w_corr = w_meas - self.bg
        a_corr = a_meas - self.ba

        q_dot = w_corr
        v_dot = self.q.apply(a_corr) + self.g
        p_dot = self.v

        return {'p': p_dot, 'v': v_dot, 'q': q_dot}
    
    def predict(self, w, a, dt) -> State:
        if self.integrator == IntegrationMethod.EULER:
            return integrate_euler(self, w, a, dt)
        elif self.integrator == IntegrationMethod.MIDPOINT:
            return integrate_midpoint(self, w, a, dt)
        elif self.integrator == IntegrationMethod.RK4:
            return integrate_rk4(self, w, a, dt)
        else:
            raise ValueError("Unknown integration method")

def integrate_euler(state, w, a, dt) -> State:
    new_state = state.copy()
    
    deriv = state.dynamics(w, a)
    new_state.p = state.p + dt * deriv['p']
    new_state.v = state.v + dt * deriv['v']
    dq = R.from_rotvec(deriv['q'] * dt)
    new_state.q = state.q * dq
    
    return new_state

def integrate_midpoint(state, w, a, dt) -> State:
    new_state = state.copy()
    
    deriv1 = state.dynamics(w, a)

    temp_state = State(
        state.p + 0.5*dt*deriv1['p'],
        state.v + 0.5*dt*deriv1['v'],
        state.q * R.from_rotvec(0.5*dt*deriv1['q']),
        state.bg, state.ba, state.g
    )

    deriv2 = temp_state.dynamics(w, a)

    new_state.p = state.p + dt * deriv2['p']
    new_state.v = state.v + dt * deriv2['v']
    dq = R.from_rotvec(deriv2['q'] * dt)
    new_state.q = state.q * dq
    
    return new_state

def integrate_rk4(state, w, a, dt) -> State:
    s = state
    
    k1 = s.dynamics(w, a)
    
    q2 = s.q * R.from_rotvec(k1['q'] * (dt/2))
    s2 = State(s.p + (dt/2)*k1['p'],
               s.v + (dt/2)*k1['v'],
               q2,
               s.bg, s.ba, s.g)
    k2 = s2.dynamics(w, a)
    
    q3 = s.q * R.from_rotvec(k2['q'] * (dt/2))
    s3 = State(s.p + (dt/2)*k2['p'],
               s.v + (dt/2)*k2['v'],
               q3,
               s.bg, s.ba, s.g)
    k3 = s3.dynamics(w, a)
    
    q4 = s.q * R.from_rotvec(k3['q'] * dt)
    s4 = State(s.p + dt*k3['p'],
               s.v + dt*k3['v'],
               q4,
               s.bg, s.ba, s.g)
    k4 = s4.dynamics(w, a)

    new_state = state.copy()
    
    new_state.p = s.p + dt*(k1['p'] + 2*k2['p'] + 2*k3['p'] + k4['p'])/6
    new_state.v = s.v + dt*(k1['v'] + 2*k2['v'] + 2*k3['v'] + k4['v'])/6
        
    dq = R.from_rotvec(
        dt*(k1['q'] + 2*k2['q'] + 2*k3['q'] + k4['q'])/6
    )
    new_state.q = s.q * dq
    
    return new_state

