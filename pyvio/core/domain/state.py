from __future__ import annotations
import numpy as np
from typing import Optional, Dict, Self
from scipy.spatial.transform import Rotation as R
import time
import struct

from pyvio.core.ports.sample import SamplePort, SampleType

G = 9.81
STATE_STRUCT = "<Q" + "fff" + "fff" + "ffff" + "fff" + "fff" + "fff"


class State(SamplePort):
    def __init__(
        self,
        timestamp: int = 0,
        position: Optional[np.ndarray] = None,
        velocity: Optional[np.ndarray] = None,
        orientation: Optional[R] = None,
        bias_g: Optional[np.ndarray] = None,
        bias_a: Optional[np.ndarray] = None,
        g: Optional[np.ndarray] = None,
    ):

        self.timestamp = timestamp
        self.p = np.zeros(3) if position is None else np.array(position)
        self.v = np.zeros(3) if velocity is None else np.array(velocity)
        self.q = R.identity() if orientation is None else orientation
        self.bg = np.zeros(3) if bias_g is None else np.array(bias_g)
        self.ba = np.zeros(3) if bias_a is None else np.array(bias_a)
        self.g = np.array([0, 0, -G]) if g is None else np.array(g)

    @classmethod
    def default(cls):
        """Return a default zero state"""
        return cls(
            timestamp=time.time_ns(),
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=R.identity(),
            bias_g=np.zeros(3),
            bias_a=np.zeros(3),
            g=np.array([0, 0, -G]),
        )

    @classmethod
    def from_vector(cls, x: np.ndarray):
        t = x[0]
        p = x[1:4]
        v = x[4:7]
        q = R.from_quat(x[7:11])
        bg = x[11:14]
        ba = x[14:17]
        g = x[17:20]

        return cls(t, p, v, q, bg, ba, g)

    def as_array(self) -> np.ndarray:
        """
        Flatten the state into a 18-element vector:
        [p(3), v(3), q(4), bias_g(3), bias_a(3), g(3)]
        """
        t_vec = (self.timestamp,)
        p_vec = self.p
        v_vec = self.v
        q_vec = self.q.as_quat()
        bg_vec = self.bg
        ba_vec = self.ba
        g_vec = self.g

        return np.concatenate([t_vec, p_vec, v_vec, q_vec, bg_vec, ba_vec, g_vec])

    def copy(self) -> State:
        """
        Return a full copy of the current state.
        """
        return State(
            timestamp=self.timestamp,
            position=self.p.copy(),
            velocity=self.v.copy(),
            orientation=self.q,  # Rotation objects are immutable-safe
            bias_g=self.bg.copy(),
            bias_a=self.ba.copy(),
            g=self.g.copy(),
        )

    def to_bytes(self) -> bytes:
        return struct.pack(
            STATE_STRUCT,
            int(self.timestamp),
            *self.p,
            *self.v,
            *self.q.as_quat(),
            *self.bg,
            *self.ba,
            *self.g,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> Self:
        unpacked = struct.unpack(STATE_STRUCT, data)

        t = unpacked[0]
        p = np.array(unpacked[1:4])
        v = np.array(unpacked[4:7])
        q = R.from_quat(unpacked[7:11])
        bg = np.array(unpacked[11:14])
        ba = np.array(unpacked[14:17])
        g = np.array(unpacked[17:20])

        return cls(
            timestamp=t,
            position=p,
            velocity=v,
            orientation=q,
            bias_g=bg,
            bias_a=ba,
            g=g,
        )

    def normalize_orientation(self):
        self.q = R.from_quat(self.q.as_quat() / np.linalg.norm(self.q.as_quat()))

    @classmethod
    def sample_size(cls) -> int:
        return struct.calcsize(STATE_STRUCT)

    @classmethod
    def sample_type(cls) -> SampleType:
        return SampleType.STATE

    def dynamics(self, w_meas: np.ndarray, a_meas: np.ndarray) -> Dict[str, np.ndarray]:
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

        return {"p": p_dot, "v": v_dot, "q": q_dot}
