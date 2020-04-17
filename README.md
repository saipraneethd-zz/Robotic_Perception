# Robotic_Perception
 Ros package to combine odometry coming from two different sensors usinge Extended Kalman filter
## Run ekf
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
## Subscription:
The node launched above subscibes to follwoing topics:
/unity_ros/husky/TrueState/odom  --->  first odometry topic
/odom2  ---> 2nd odometry topic (not tested yet)

## Publication
The node launched above publishes output odometry on following topic:
/odom_filtered
