import rospy
import numpy as np
from ekf import EKF
from scipy import random
from nav_msgs.msg import Odometry
from threading import Thread, Lock

class Filter:
    """Implements general filter functionalities"""

    def __init__(self):
        self.state_dim = rospy.get_param('state_dim', 4)
        self.control_dim = rospy.get_param('control_dim', 2)
        self.measurement_dim = rospy.get_param('measurement_dim', 4)
        self.frame_id = rospy.get_param('frame_id', "/filter_frame")

        Q = rospy.get_param('process_noise', [0.1]*self.state_dim)
        P_init = rospy.get_param('init_covar', [10.0]*self.state_dim)
        R1 = rospy.get_param('R1', [0.1]*self.measurement_dim)
        R2 = rospy.get_param('R2', [0.1]*self.measurement_dim)
        X_init = rospy.get_param('X_init', [0., 0., 0., 0.])
        feq = rospy.get_param('filter_freq', 10)
        out_odom_topic = rospy.get_param('out_odom_topic', "/odom_filtered")
        odom1_topic = rospy.get_param('odom1_topic', "/odom1")
        odom2_topic = rospy.get_param('odom2_topic', "/odom2")

        self.Q = np.diag(Q)
        self.P_init = np.diag(P_init)
        self.R1 = np.diag(R1)
        self.R2 = np.diag(R2)
        self.X_init = np.array(X_init)
        self.r = rospy.Rate(feq)

        self.A = np.identity(self.state_dim)
        self.H = np.identity(self.state_dim)
        self.B = np.zeros(1)
        self.U = np.zeros(1)

        self.mutex = Lock()
        self.last_update_time = None

        self.filt = EKF(self.X_init, self.Q, self.P_init)

        self.odom_pub = rospy.Publisher(out_odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(odom1_topic, Odometry, self.odom1_callback)
        rospy.Subscriber(odom2_topic, Odometry, self.odom2_callback)

    def odom1_callback(self, data):
        self.mutex.acquire()
        self.odom_update(data, self.R1, "odom1")
        self.mutex.release()

    def odom2_callback(self,data):
        self.mutex.acquire()
        self.odom_update(data, self.R2, "odom2")
        self.mutex.release()

    def odom_update(self, data, R, source):
        if self.last_update_time is None:
            self.last_update_time = rospy.Time.now()
        current_time = rospy.Time.now()
        #delt = current_time - self.last_update_time
        delt = (data.header.stamp - self.last_update_time).to_sec()
        #self.last_update_time = rospy.Time.now()
        self.A[0][2] = delt
        self.A[1][3] = delt
        if delt < .05:
            return
        self.last_update_time = data.header.stamp
        rospy.logdebug("\nreceived %s , time elapsed since last update %s\n",source,  delt)
        rospy.logdebug("\nbefore %s prediction\n state = \n%s\ncov = \n%s\n", source, self.filt.X, self.filt.P)
        self.filt.predict(self.A, self.B, self.U)
        rospy.logdebug("\nafter prediction\nstate = \n%s\ncov = \n%s\n",self.filt.X, self.filt.P)
        Z = self.get_z_from_odom(data)
        self.filt.update(Z, self.H, R)
        rospy.logdebug("\ngot %s measurement\nstate = \n%s\n cov = \n%s\n", source, Z,R)
        rospy.logdebug("\nafter %s update\nstate = \n%s\ncov = \n%s\n\n\n\n", source, self.filt.X, self.filt.P)
        self.publish_odom_from_z(self.filt.X)

    def get_z_from_odom(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y
        Z = np.array([x, y, vx, vy])
        return Z

    def publish_odom_from_z(self,Z):
        odom = Odometry()
        odom.pose.pose.position.x = Z[0]
        odom.pose.pose.position.y = Z[1]
        odom.twist.twist.linear.x = Z[2]
        odom.twist.twist.linear.y = Z[3]
        odom.header.frame_id = self.frame_id
        odom.header.stamp = rospy.Time.now()
        self.odom_pub.publish(odom)
        """R = np.zeros([state_dim, state_dim])
        #pose covariance
        pcv = data.pose.covariance
        #twist covariance
        tcv = data.twist.covariance
        for i in range(0,2):
            for j in range(0,2):
                R[i][j] = pcv[2*i + j]
        for i in range(2,4):
            for j in range(2,4):
                R[i][j] = tcv[4*i + j]
        """

