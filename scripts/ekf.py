import numpy as np

class EKF:
    """Implements Extended Kalman filter

    update equations:
        X_t = AX_{t-1} + BU_{t-1}
        Z_t = HX_t
        Q: process noise
        R: measurement noise
    """
    def __init__(A, B, H, Q, R):
        """Initialze EKF with A, B, H, Q and R"""
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
    def predict(X, U, P):
        X = A@X + B@U
        P = A@P@A.transpose() + Q


