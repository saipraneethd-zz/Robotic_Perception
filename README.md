# MEEN 689 - Robotic Perception

​		This project compares Least-squares estimator(LSQ) and Extended Kalman Filter(EKF) for sensor fusion using Odometry data collected from simulated environment.

## Simulation Environment ##

​		This project uses [AirSim](https://github.com/Microsoft/AirSim), an open-source, and cross platform simulation environment for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/) (we now also have an experimental [Unity](https://unity3d.com/) release). It supports collecting simulated data from various sensors such as Camera (Including Depth ), LIDAR, GPS, IMU, etc. The vehicles(car/drone) can be controlled manually with keyboard or remote control (RC). Airsim also provides pre-built [Python API](https://airsim-fork.readthedocs.io/en/docs/apis.html) to interact with vehicles such as to retrieve images, get current state of vehicle, and control the vehicle.

​		For the  current simulation we used pre-built of [Modular Neighborhood Pack](https://www.unrealengine.com/marketplace/en-US/product/modular-neighborhood-pack), available to download for free at [AirSim Releases - v1.2.2 - Windows](https://github.com/microsoft/AirSim/releases/download/v1.3.0-Windows/Neighborhood.zip).
​	![](/assets/airsim_snap.png)

## Setup ##

The main packages used are
* AirSim
* Matplotlib
* Numpy
* Pandas
* Scipy

The packages can be installed by running
```sh
  pip install -r requirements.txt
```
(Ensure pip corresponds to Python 3.x, alternatively use pip3)

Before launching Airsim, the `setting.json` in `Documents/Airsim/ ` should be replaced with `/assets/settings.json`. Launching `AirSimNH.exe` starts the simulator which looks as follows

## Data Collection ##

Start the simulation environment by running `AirSimNH.exe` (In this case neighborhood environment, other environments can also be used). To collect trajectory data run `/scripts/generate_traj.py` while running the Airsim environment, which connects to `CarClient()` and starts recording the GPS and Image data.

We collected three runs of data presented in the `data` folder. The first run includes the car following a straight line and the second and third runs include the car making a loop around the neighborhood. Each run folder consists of `raw_images`, `ground_truth.csv`, `gauss_noise1.csv`, `gauss_noise2.csv` which represent the raw image data files collected during the run, ground truth consisting `[x,y,vx,vy]` and noisy measurements of two GPS sensors with gaussian noise.

```sh
python generate_traj.py
```

### Straight Line Run ###

Data is available at `/data/straight_line`

![](/assets/straight_line.gif)

### Loop Run ###

The first run data is available at `/data/loop` and the second  run data is available at `/data/loop2`
![](/assets/loop.gif)

## LSQ Estimator ##

Linear Least Squares estimator calculates the estimates of the values of position and velocity using the measurements of the two different Sensors

Run the linear least squares by running the least_squares.py file in scripts folder

```sh
  python scripts/least_squares.py
```

## EKF Estimator ##

Extended Kalman filter (EKF) is the nonlinear version of the Kalman filter which linearizes about an estimate of the current mean and covariance. The python file `/scripts/ekf.py` consists of `EKF` class, which can be tested using the following

```sh
  python scripts/ekf_test.py
```


## Comparison of LSQ and Linear EKF Estimators
Run the following code to compare both the non-linear estimators.

```sh
  cd scripts/
  jupyter notebook
```

Open `Compare_lsq_ekf.ipynb` and run all the cells

## Comparison of NLSQ and Non-Linear EKF Estimators
Run the following code to compare both the non-linear estimators.

```sh
  cd scripts/
  jupyter notebook
```

Open `Compare_nlsq_ekf.ipynb` and run all the cells
