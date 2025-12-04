import numpy as np

"""
    Usage
    
    # State: [alpha, beta]
    N = 2

    A = np.eye(N)          # random walk
    Q = np.eye(N)*1e-6
    x0 = np.array([1.0, 0.0])
    P0 = np.eye(N)*1e-3

    kf = KalmanFilter(x0, P0, A, Q)

    # Run
    for t1, t2 in zip(interrupt_t1, interrupt_t2):
        H = np.array([[t1, 1.0]])   # [1x2]
        R = np.array([[1e-4]])      # [1x1]
        kf.predict()
        kf.update(z=[t2], H=H, R=R)
        print(kf.x)

"""


class KalmanFilter:
    def __init__(self, x0, P0, A, Q):
        """
        Generic linear Kalman filter.

        x0 : [N] initial state
        P0 : [NxN] initial covariance
        A  : [NxN] state transition
        Q  : [NxN] process noise
        """
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.A = np.array(A, dtype=float)
        self.Q = np.array(Q, dtype=float)

    def predict(self):
        """x = A x ; P = A P A^T + Q"""
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z, H, R):
        """
        z : [M] measurement vector
        H : [MxN] measurement matrix
        R : [MxM] measurement noise
        """
        z = np.array(z, dtype=float)
        H = np.array(H, dtype=float)
        R = np.array(R, dtype=float)

        # Innovation
        y = z - (H @ self.x)

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman Gain: K = P H^T S^{-1}
        # Efficient solve: solve S K^T = (H P)^T
        K = np.linalg.solve(S, (H @ self.P).T).T

        # Update
        self.x = self.x + K @ y
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P
