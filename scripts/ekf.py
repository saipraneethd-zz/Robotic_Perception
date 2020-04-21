import numpy as np
from scipy import linalg as spla
import math
import matplotlib.pyplot as plt

class EKF:
    """Implements Extended Kalman filter

    predict equations:
        X_bar_t = A_t@X_{t-1} + B_t@U_{t-1}
        P_bar_t = A@P_{t-1}@A^T + Q

        Q: process noise
    update equations:
        Y_t = Z_t - H_t@X_bar_t
        S_t = H_t@P_bar_t@H_t^T + R
        K_t = P_bar_t@H_t^T@(S_t)^(-1)
        X_t = X_bar_t + K_t@Y_T
        P_t = (I - K_t@H_t)@P_bar_t

        R: measurement noise
    """
    def __init__(self, X, Q, P):
        """Initialze EKF with:
            X: initial state
            P: initial covariance
            Q: process Noise
        """

        self.X = X
        self.P = P
        self.Q = Q
        alpha = np.arange(0, 2*math.pi, 0.01)
        x  = np.cos(alpha)
        y = np.sin(alpha)
        self.circle = np.array([x,y])
    def predict(self, A, B, U):
        """Prediction for Kalman filter assuming variable time step
        A, B will depend on time
        """

        self.X = A@self.X + B@U
        self.P = A@self.P@A.transpose() + self.Q
    def update(self, Z, H, R):
        """ Measurement update for Kalman filter
        different sensors will have different H
        and different sensor Noise R
        """

        H_T = H.transpose()

        Y = Z - H@self.X
        S = H@self.P@H_T + R

        #stable inverse calculation
        L = np.linalg.cholesky(S)
        L_inv = spla.solve_triangular(L, np.identity(L.shape[0]), lower=True)
        S_inv = L_inv.transpose() @ L_inv

        K = self.P@H_T@S_inv
        self.X = self.X + K@Y
        self.P = (np.identity(self.X.shape[0]) - K@H)@self.P
    def nl_update(self, Z, H, R, Z_bar):
        """ Measurement update for Kalman filter
        different sensors will have different H
        and different sensor Noise R
        """

        H_T = H.transpose()

        Y = Z - Z_bar
        angle = Y[1]
        Y[1] = (angle + math.pi) % (2 * math.pi) - math.pi
        S = H@self.P@H_T + R

        #stable inverse calculation
        L = np.linalg.cholesky(S)
        L_inv = spla.solve_triangular(L, np.identity(L.shape[0]), lower=True)
        S_inv = L_inv.transpose() @ L_inv

        K = self.P@H_T@S_inv
        self.X = self.X + K@Y
        self.P = (np.identity(self.X.shape[0]) - K@H)@self.P
    def plot_cov(self, X, P, label):
        """Plot the 2d error Ellipse given mean X and Covariance P"""

        X = X[0:2]
        P = P[0:2, 0:2]

        L = np.linalg.cholesky(P)
        ellipse = L@self.circle
        plt.plot(ellipse[0,:] + X[0], ellipse[1,:] + X[1], label=label)
        w, v = np.linalg.eig(P)
        origin = X
        plt.arrow(*origin, *(math.sqrt(w[0])*v[:,0]))
        plt.arrow(*origin, *(math.sqrt(w[1])*v[:,1]))
        plt.axis("equal")
        plt.plot()





