import rospy
import numpy as np
from ekf import EKF
from scipy import random

A = random.rand(2, 2)
A = np.dot(A, A.transpose())

X = np.zeros(2)

ekf = EKF(X, 0, 0)
ekf.plot_cov(X, A)
