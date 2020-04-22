import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from ekf import EKF


def estimate_x(meas1, meas2, ground_truth, R1, R2, H):
    z1 = meas1.T
    z2 = meas2.T
    x = (0.5*(np.linalg.inv((H.T)@np.linalg.inv(R1)@H)@(H.T)@(np.linalg.inv(R1))@z1)\
         + 0.5 * (np.linalg.inv((H.T)@np.linalg.inv(R2)@H)@(H.T)@(np.linalg.inv(R2))@z2)).T
    # for i in range(len(x)):
        # 	print('Estimated x: {}, Ground truth: {}'.format(x[i], ground_truth[i]))
    plot_x(x, ground_truth, "lsq")
    print(f"lsq ---> pose error = {np.linalg.norm(x-ground_truth)}")
def plot_x(x, ground_truth, algo):
    plt.figure()
    plt.title(f"{algo} Position Comparison")
    plt.plot(x[:, 0], x[:, 1], label = f"{algo} Estimate")
    plt.plot(ground_truth[:, 0], ground_truth[:, 1], label = f"{algo} Ground Truth")
    plt.xlabel("Position X")
    plt.ylabel("Position Y")
    plt.legend()
    plt.savefig(f"{algo} Position Comparison")
    plt.figure()
    plt.title(f"{algo} Velocity Comparison")
    plt.plot(x[:, 2], x[:, 3], label = f"{algo} Estimate")
    plt.plot(ground_truth[:, 2], ground_truth[:, 3], label = "Ground Truth")
    plt.xlabel("Velocity X")
    plt.ylabel("Velocity Y")
    plt.legend()
    plt.savefig(f"{algo} Velocity Comparison")
def ekf_estimate(meas1, meas2, ground_truth, R1, R2, H, Q):
    delt = 0.1
    state_dim = 4
    X_init = np.array([0., 0., 0., 0.])
    P_init = np.diag([10.,10., 10., 10.])
    n_obs = meas1.shape[0]
    A = np.identity(state_dim)
    A[0][2] = delt
    A[1][3] = delt
    A[0][2] = 0.
    A[1][3] = 0.
    B = np.zeros(1)
    U = np.zeros(1)
    filt1 = EKF(X_init, Q, P_init)
    filt2 = EKF(X_init, Q, P_init)
    x1 = np.zeros([n_obs, 4])
    x2 = np.zeros([n_obs, 4])
    for i in range(0, n_obs):
        filt1.predict(A, B, U)
        filt1.update(meas1[i], H, R1)
        filt1.update(meas2[i], H, R2)
        x1[i] = filt1.X
        filt2.X = meas1[i]
        filt2.P = R1
        filt2.update(meas2[i], H, R2)
        x2[i] = filt2.X
    plot_x(x1, ground_truth, f"kf with prediction, Q = 0_01")
    plot_x(x2, ground_truth, "kf without prediction")
    print(f"kf with predction ---> Q = diag({Q[0][0]}_0), pose error = {np.linalg.norm(x1-ground_truth)}")
    print(f"kf without predction ---> pose error = {np.linalg.norm(x2-ground_truth)}")

def main():
    win_size = 1
    R1 = np.identity(4)
    R2 = np.identity(4)*0.5
    H = np.identity(4)
    Q = np.diag([0.01, 0.01, 0.01, 0.01])
    gn1 = pd.read_csv('../data/straight_line/gauss_noise1.csv', sep = ',')
    gn2 = pd.read_csv('../data/straight_line/gauss_noise2.csv', sep = ',')
    gt = pd.read_csv('../data/straight_line/ground_truth.csv', sep = ',')
    meas1 = gn1.to_numpy()[:, 2:]
    meas2 = gn2.to_numpy()[:, 2:]
    ground_truth = gt.to_numpy()[:, 2:]
    rows_consider = np.arange(0,meas1.shape[0],1)
    print(rows_consider)
    meas1 = meas1[rows_consider]
    meas2 = meas2[rows_consider]
    ground_truth = ground_truth[rows_consider]
    estimate_x(meas1, meas2, ground_truth, R1, R2, H)
    ekf_estimate(meas1, meas2, ground_truth, R1, R2, H, Q)

main()
