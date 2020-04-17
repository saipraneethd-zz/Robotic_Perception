# Robotic_Perception
 Ros package to combine odometry coming from two different sensors usinge Extended Kalman filter
## Start ekf
To start ekf run the following command:
```sh
roslaunch Robot_Perception ekf_node.launch
```
## Subscription:
The node launched above subscibes to follwoing topics:
\odom1  --->  first odometry topic
\odom2  ---> 2nd odometry topic

## Publication
The node launched above publishes output odometry on following topic:
\odom_filtered
