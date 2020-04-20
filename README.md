# MEEN 689 - Robotic Perception

​		This project compares Least-squares estimator(LSQ) and Extended Kalman Filter(EKF) for sensor fusion using Odometry data collected from simulated environment.

## Simulation Environment ##

​		This project uses [AirSim](https://github.com/Microsoft/AirSim), an open-source, and cross platform simulation environment for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/) (we now also have an experimental [Unity](https://unity3d.com/) release). It supports collecting simulated data from various sensors such as Camera (Including Depth ), LIDAR, GPS, IMU, etc. The vehicles(car/drone) can be controlled manually with keyboard or remote control (RC). Airsim also provides pre-built [Python API](https://airsim-fork.readthedocs.io/en/docs/apis.html) to interact with vehicles such as to retrieve images, get current state of vehicle, and control the vehicle.

​		For the  current simulation we used pre-built of [Modular Neighborhood Pack](https://www.unrealengine.com/marketplace/en-US/product/modular-neighborhood-pack), available to download for free at [AirSim Releases - v1.2.2 - Windows](https://github.com/microsoft/AirSim/releases/download/v1.3.0-Windows/Neighborhood.zip). Before launching Airsim, the `setting.json` in `Documents/Airsim/ ` should be replaced with `/assets/settings.json`. Launching `AirSimNH.exe` starts the simulator which looks as follows

​	![](/assets/airsim_snap.png)

## Data Collection ##

To collect trajectory data run `/scripts/generate_traj.py`, which connects to `CarClient()` and starts

recording the GPS and Image data. We collected two runs of data presented in the `data` folder. The first run includes the car following a straight line and the second run includes the car making a loop around the neighborhood. Each run folder consists of `raw_images`, `ground_truth.csv`, `gauss_noise1.csv`, `gauss_noise2.csv` which represent the raw image data files collected during the run, ground truth consisting `[x,y,vx,vy]` and noisy measurements of two GPS sensors with gaussian noise.

```sh
python generate_traj.py
```

### Straight Line Run ###

Data is available at `/data/straight_line`

![](/assets/straight_line.gif)

### Loop Run ###

Data is available at `/data/loop`

![](/assets/loop.gif)

## LSQ Estimator ##

## EKF Estimator ##

Ros package to combine odometry coming from two different sensors using Extended Kalman filter

### Run EKF ###

- Start roscore
     ```sh
    roscore
    ```
- In a new terminal run the following command to activate simulation time
    ```sh
    rosparam set use_sim_time true
    ```

- In a new terminal run the bag file (only tested with one odometry as of now)
    ```sh
    rosbag play data/2020-04-17-03-55-13.bag --clock
    ```
- In a new terminal start ekf:
    ```sh
    roslaunch Robot_Perception ekf_node.launch
    ```

- View input odometry in a new terminal:
   ```sh
   rostopic echo /unity_ros/husky/TrueState/odom
   ````

- View the filter output odometry in a new terminal:
   ```sh
   rostopic echo /odom_filtered
   ````

- In addition to this the ekf node will also print debugging messages on stdout
### Subscribers
The node launched above subscribes to following topics:
   ```sh
/unity_ros/husky/TrueState/odom  --->  first odometry topic
  ````
  ```sh
/odom2  ---> 2nd odometry topic (not tested yet)

   ````
### Publishers
The node launched above publishes output odometry on following topic:
   ```sh
/odom_filtered

   ````
