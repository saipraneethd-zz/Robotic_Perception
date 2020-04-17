import numpy as np
from ekf import EKF
from scipy import random

import matplotlib.pyplot as plt

#initialize ekf
"""using model:
    x_t = x_{t-1} + vx_{t-1} * dt
    y_t = y_{t-1} + vy_{t-1} * dt
    vx_t = vx_{t-1}
    vy_t = vy_{t-1}

    A = [[1, 0, dt, 0],
         [0, 1, 0, dt],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]
    B = 0 (no control)
"""
#initial cov
P = np.diag([0.4, 0.3, 0.01, 0.01])
#state (x, y , vx, vy)
X = np.array([1,1,1,1])
#process noise
Q = np.diag([0.1 , 0.1, 0.1, 0.1])

ekf = EKF(X, Q, P)

#plot initial position and error ellipse
label = "initial state and cov"
ekf.plot_cov(ekf.X, ekf.P, label)

#test prediction
dt = 0.1

A = np.array([[1, 0, dt, 0],\
              [0, 1, 0, dt],\
              [0, 0, 1, 0],\
              [0, 0, 0, 1]])
#test to check if prediction is working. error ellipses should grow with time
for i in range(0,10):
    ekf.predict(A, np.array([0]), np.array([0]))
    ekf.plot_cov(ekf.X, ekf.P, label)
plt.title("Prediction step test, error grows with time if there is no measurement update")

fig = plt.figure()
label = "state and error after after 10 prediction steps"
ekf.plot_cov(ekf.X, ekf.P, label)
#time step


#simulate measurement
R = np.diag([0.4, 0.3, 0.01, 0.01])
Z = np.array([3.5 ,3.5 ,2.2 ,2.3])
H = np.identity(4) #simple case assuming we can observe complete state 
label = "measurement and covariance"
ekf.plot_cov(Z, R, label)


#do measurement update
ekf.update(Z, np.identity(4), R)

label = "state and cov after update"
ekf.plot_cov(ekf.X, ekf.P, label)
plt.legend(loc="lower right")
plt.show()
