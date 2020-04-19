import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def estimate_x(meas1, meas2, ground_truth, R1, R2, H):
	z1 = meas1.T
	z2 = meas2.T
	x = (0.5*(np.linalg.inv((H.T)@np.linalg.inv(R1)@H)@(H.T)@(np.linalg.inv(R1))@z1)\
	+ 0.5 * (np.linalg.inv((H.T)@np.linalg.inv(R2)@H)@(H.T)@(np.linalg.inv(R2))@z2)).T
	# for i in range(len(x)):
	# 	print('Estimated x: {}, Ground truth: {}'.format(x[i], ground_truth[i]))
	plt.figure()
	plt.title("Position Comparison")
	plt.plot(x[:, 0], x[:, 1], label = "Least Squares Estimate")
	plt.plot(ground_truth[:, 0], ground_truth[:, 1], label = "Ground Truth")
	plt.xlabel("Position X")
	plt.ylabel("Position Y")
	plt.legend()
	plt.savefig("Position Comparison")
	plt.figure()
	plt.title("Velocity Comparison")
	plt.plot(x[:, 2], x[:, 3], label = "Least Squares Estimate")
	plt.plot(ground_truth[:, 2], ground_truth[:, 3], label = "Ground Truth")
	plt.xlabel("Velocity X")
	plt.ylabel("Velocity Y")
	plt.legend()
	plt.savefig("Velocity Comparison")

def main():
	win_size = 1
	R1 = np.identity(4)
	R2 = np.identity(4)*0.5
	H = np.identity(4)
	gn1 = pd.read_csv('gauss_noise1.csv', sep = ',')
	gn2 = pd.read_csv('gauss_noise2.csv', sep = ',')
	gt = pd.read_csv('ground_truth.csv', sep = ',')
	meas1 = gn1.to_numpy()[:, 2:]
	meas2 = gn2.to_numpy()[:, 2:]
	ground_truth = gt.to_numpy()[:, 2:]
	estimate_x(meas1, meas2, ground_truth, R1, R2, H)

main()