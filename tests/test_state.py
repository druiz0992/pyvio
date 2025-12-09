import numpy as np
from scipy.spatial.transform import Rotation as R

from pyvio.core.domain.state import (
    State,
    IntegrationMethod,
    integrate_euler,
    integrate_midpoint,
    integrate_rk4
)

# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------

def assert_vec_close(a, b, tol=1e-6):
    assert np.allclose(a, b, atol=tol), f"{a} != {b}"

def assert_quat_close(q1: R, q2: R, tol=1e-6):
    # Compare rotation matrices
    assert np.allclose(q1.as_matrix(), q2.as_matrix(), atol=tol)

# ------------------------------------------------------------
# Tests
# ------------------------------------------------------------

def test_default_state():
    s = State.default()
    
    assert_vec_close(s.p, np.zeros(3))
    assert_vec_close(s.v, np.zeros(3))
    assert_quat_close(s.q, R.identity())
    assert_vec_close(s.bg, np.zeros(3))
    assert_vec_close(s.ba, np.zeros(3))
    assert_vec_close(s.g, np.array([0,0,-9.81]))


def test_to_from_vector_roundtrip():
    s = State(
        position=np.array([1,2,3]),
        velocity=np.array([0.1, 0.2, 0.3]),
        orientation=R.from_euler("xyz", [10, 20, 30], degrees=True),
        bias_g=np.array([0.01,0.02,0.03]),
        bias_a=np.array([0.1,0.2,0.3]),
        g=np.array([0,0,-9.81])
    )

    vec = s.to_vector()
    s2 = State.from_vector(vec)

    assert_vec_close(s2.p, s.p)
    assert_vec_close(s2.v, s.v)
    assert_vec_close(s2.bg, s.bg)
    assert_vec_close(s2.ba, s.ba)
    assert_quat_close(s2.q, s.q)


def test_dynamics_zero_motion():
    s = State.default()
    w = np.zeros(3)
    a = np.array([0,0,9.81])  # cancels gravity

    d = s.dynamics(w, a)

    # p_dot = v = 0
    assert_vec_close(d['p'], np.zeros(3))
    # v_dot = R*(a - ba) + g = +9.81 + (-9.81) = 0
    assert_vec_close(d['v'], np.zeros(3))
    # q_dot = w - bg = 0
    assert_vec_close(d['q'], np.zeros(3))


def test_integrate_euler_simple_motion():
    s = State.default()
    w = np.zeros(3)
    a = np.array([0, 0, 1.0])
    dt = 1.0

    s2 = integrate_euler(s, w, a, dt)

    # v = a + g = 1 - 9.81 = -8.81
    expected_v = np.array([0, 0, -8.81])
    assert_vec_close(s2.v, expected_v)

    # p = v*dt = same value
    expected_p = np.array([0, 0, 0])
    assert_vec_close(s2.p, expected_p)


def test_integrate_euler_rotation():
    s = State.default()
    w = np.array([0, 0, np.deg2rad(90)])  # 90 deg/s yaw
    a = np.zeros(3)
    dt = 1.0

    s2 = integrate_euler(s, w, a, dt)

    expected_q = R.from_euler("z", 90, degrees=True)
    assert_quat_close(s2.q, expected_q)


def test_midpoint_close_to_euler_for_small_dt():
    s = State.default()
    w = np.array([0.1, -0.2, 0.3])
    a = np.array([0.5, 0.2, -0.1])
    dt = 0.001

    s_euler = integrate_euler(s, w, a, dt)
    s_mid = integrate_midpoint(s, w, a, dt)

    assert_vec_close(s_euler.p, s_mid.p, tol=1e-4)
    assert_vec_close(s_euler.v, s_mid.v, tol=1e-4)
    assert_quat_close(s_euler.q, s_mid.q, tol=1e-4)


def test_rk4_more_accurate_than_euler():
    s = State.default()
    w = np.array([0.3, 0.2, -0.4])
    a = np.array([1.0, 0.1, 0.2])
    dt = 0.1

    s_euler = integrate_euler(s, w, a, dt)
    s_rk4 = integrate_rk4(s, w, a, dt)

    # rk4 should differ from euler (more accurate)
    assert not np.allclose(s_euler.p, s_rk4.p)
    assert not np.allclose(s_euler.v, s_rk4.v)
