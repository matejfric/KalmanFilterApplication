from dataclasses import dataclass
import numpy as np
from scipy.linalg import inv

"""
Simple implementation of Kalman filter.

Inspired by: Roger R. Labbe - Kalman and Bayesian Filters in Python
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
"""

@dataclass
class KF:
    F: np.ndarray         # Nx x Nx (state-transition matrix)
    P: np.ndarray         # Nx x Nx (covariance matrix)
    Q: np.ndarray         # Nx x Nx (process noise matrix)
    H: np.ndarray         # Ny x Nx (observation matrix)
    R: np.ndarray         # Nu x Nu (measurement error matrix)
    x: np.ndarray         # Nx x 1 (state vector)
    zs: np.ndarray        # N (measurements)
    B: np.ndarray = None  # Nx x Nu (input control matrix)
    u: np.ndarray = None  # Nf x 1 (input vector)
    dt: float = None      # 1 (time step / delta time)

    def __post_init__(self):
        if self.B is None:
            self.B = np.zeros(self.F.shape)
        if self.u is None:
            self.u = np.zeros(self.F.shape[0])
        if self.dt is None:
            self.dt = 1

    def get_fields(self):
        F = self.F
        P = self.P
        Q = self.Q
        H = self.H
        R = self.R
        B = self.B
        u = self.u
        x = self.x
        zs = self.zs
        return F, P, Q, H, R, B, u, x, zs

    def run(self):
        F, P, Q, H, R, B, u, x, zs = self.get_fields()
        xs = [x] # state vectors x_k
        cov = [P] # covariance matrices P_k
        ks = [] # Kalman gain matrices K_k

        for k,z in enumerate(zs):
            # Predict
            if callable(u):
                time = k * self.dt
                x = F @ x + B @ u(time)
            else:
                x = F @ x + B @ u
            P = F @ P @ F.T + Q

            # Update
            S = H @ P @ H.T + R
            K = P @ H.T @ inv(S)  # Kalman gain
            y = z - H @ x  # Inovation
            x += K @ y
            P = P - K @ H @ P

            xs.append(x)
            cov.append(P)
            ks.append(K)

        return np.array(xs), np.array(cov), np.array(ks)
    