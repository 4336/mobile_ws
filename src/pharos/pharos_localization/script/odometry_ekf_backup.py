#!/usr/bin/env python
import numpy as np
from math import *

import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion

roslib.load_manifest('pharos_localization')

class RosGpsEkf:
    """docstring for RosGpsEkf"""

    odom_ekf_pub = rospy.Publisher('/odom/slam_ekf', Odometry)
    slam_odom_pub = rospy.Publisher('/onlyslam', Odometry)
    tf_br = tf.TransformBroadcaster()

    relative_slam = False
    publish_odom_tf = True

    frame_id = ''
    child_frame_id = ''

    slam_rate = 20;

    def __init__(self):
        self.ekf_reset()

    def ekf_reset(self):
        # EKF variables
        self.Q = np.mat(np.diag([1e-1, 1e-1, 1e0]))
        self.P = np.mat(np.diag([0.0]*3))
        # 2 x 2 matrix
        self.R_meas = np.mat(np.diag([0.0]*2))
        self.F = np.mat(np.diag([1.0]*3))
        self.H = np.mat('1 0 0; 0 1 0')
        self.Z = np.mat('0.0; 0.0')
        self.X_post = np.mat('0.0; 0.0; 0.0')
        self.X_pred = np.mat('0.0; 0.0; 0.0')
        self.X_filtered = [[0.0, 0.0, 0.0]]
        self.ori_x = [[.0,.0]]
        self.ori_y = [[.0,.0]]
        # initialize variables
        self.x_meas, self.y_meas, self.x_cov, self.y_cov, self.vel, self.alpha = [0.0]*6
        self.last_time_vehicle = rospy.Time(0)
        self.last_time_slam = rospy.Time(0)
        self.filter_initialized = False
        self.old_slam_x = 0.0
        self.old_slam_y = 0.0
 
        self.origin_slam_x = 0.0
        self.origin_slam_y = 0.0

        self.last_time_state = rospy.Time(0)
        self.state = Point(0,0,0);
        self.state_old = Point(0,0,0);

        self.heading = 0.0;
        self.heading_old = 0.0;

    
    def husky_model(self, x, y, theta, vel, alpha, dt):

        d = vel * dt
        
        if abs(alpha) < 0.001:
            x = x + d * cos(theta)
            y = y + d * sin(theta)
            theta = theta + alpha
            x_dot = -d * sin(theta)
            y_dot = d * cos(theta)
        else:
            R = d / alpha
            Cx = x - sin(theta) * R
            Cy = y + cos(theta) * R
            theta = theta + alpha
            x = Cx + sin(theta) * R
            y = Cy - cos(theta) * R
            x_dot = -R * cos(theta - alpha) + R * cos(theta)
            y_dot = -R * sin(theta - alpha) + R * sin(theta)
            
        return x, y, theta, x_dot, y_dot

    def ekf_init(self):
        self.X_post = np.mat([self.x_meas, self.y_meas, 0.0]).T
        self.P[0,0] = self.x_cov
        self.P[1,1] = self.y_cov
        self.P[2,2] = 1e9
        self.R_meas[0,0] = self.x_cov
        self.R_meas[1,1] = self.y_cov

    def ekf_update(self, dt):
        new_slam_x = self.x_meas
        new_slam_y = self.y_meas

        # prediction
        x, y, theta = self.X_post[0,0], self.X_post[1,0], self.X_post[2,0]
        
        #self.X_filtered.append([x, y, theta])
        
        x, y, theta, x_dot, y_dot = self.husky_model(x, y, theta, self.vel, self.alpha, dt)

        if new_slam_x == self.old_slam_x and new_slam_y == self.old_slam_y:
            #rospy.logwarn("Wrong (duplicated) SLAM data. Extrapolating...")
            self.X_post[0,0] = x
            self.X_post[1,0] = y
            self.X_post[2,0] = theta
            return

        self.old_slam_x = new_slam_x
        self.old_slam_y = new_slam_y

        self.X_pred[0,0] = x
        self.X_pred[1,0] = y
        self.X_pred[2,0] = theta

        #self.ori_x.append([x, cos(theta)])
        #self.ori_y.append([y, sin(theta)])
        
        # model linearization
        self.F[0,2] = x_dot
        self.F[1,2] = y_dot
        
        self.P = self.F * self.P * self.F.T + self.Q
        
        self.R_meas[0,0] = self.x_cov
        self.R_meas[1,1] = self.y_cov
        
        self.Z[0,0] = self.x_meas
        self.Z[1,0] = self.y_meas
        
        self.Y = self.Z - self.X_pred[0:2]
        self.S = self.H * self.P * self.H.T + self.R_meas
        self.K = self.P * self.H.T * self.S.I
        
        self.X_post = self.X_pred + self.K * self.Y
        
        self.P = (np.eye(3) - self.K * self.H) * self.P

    def slam_odom_callback(self, msg):
        dt = (msg.header.stamp - self.last_time_slam).to_sec()

        cov = np.array(msg.pose.covariance).reshape(6,6)
        new_slam_x = msg.pose.pose.position.x
        new_slam_y = msg.pose.pose.position.y

        if not self.filter_initialized:
            self.ekf_reset()

            self.last_time_vehicle = self.last_time_slam = msg.header.stamp
            self.ekf_init()
            #self.child_frame_id = msg.header.frame_id

            # We always calculate in relative coordinates, but then displace result to global frame
            self.origin_slam_x = new_slam_x
            self.origin_slam_y = new_slam_y
            if self.relative_slam:
                rospy.set_param('~origin/x', new_slam_x)
                rospy.set_param('~origin/y', new_slam_y)
                rospy.loginfo("Using relative SLAM coordinates. Origin stored in ~/origin")

            self.old_slam_x = new_slam_x - self.origin_slam_x
            self.old_slam_y = new_slam_y - self.origin_slam_y

            self.filter_initialized = True
            rospy.loginfo("EKF initialized")
            return

        self.x_meas = new_slam_x - self.origin_slam_x
        self.y_meas = new_slam_y - self.origin_slam_y

        self.x_cov = cov[0,0]
        self.y_cov = cov[1,1]

        self.last_time_slam = msg.header.stamp
        
        self.ekf_update(dt)

    def vehicle_state_callback(self, msg):
        self.old_state = self.state;
        self.state = msg.pose.pose.position;

        global roll, pitch
        self.heading_old = self.heading;
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.heading) = euler_from_quaternion (orientation_list)

        ###
        self.vel = sqrt((self.state.x - self.old_state.x)**2 + (self.state.y - self.old_state.y)**2)
        
        slam_dt = (msg.header.stamp - self.last_time_slam).to_sec()

        dt = (msg.header.stamp - self.last_time_state).to_sec()
        self.last_time_state = msg.header.stamp;

        self.alpha = (self.heading - self.heading_old)/dt
        print dt, self.alpha

        # if  slam_dt > 0.05: #((1/self.slam_rate) * 10):
        #     if not self.filter_initialized: return
        #     rospy.logwarn('SLAM got lost, last update was ' + str(slam_dt) + 's ago')
        #     dt = (msg.header.stamp - self.last_time_vehicle).to_sec()
        #     self.last_time_vehicle = rospy.Time(0.05 + msg.header.stamp.to_sec())
        #     self.last_time_slam = rospy.Time(0.05 + msg.header.stamp.to_sec())
        #     # update with fake values
        #     x, y, theta = self.X_post[0,0], self.X_post[1,0], self.X_post[2,0]
        #     x, y, theta, x_dot, y_dot = self.bicycle_model(x, y, theta, self.vel, self.alpha, 0.05)
        #     self.X_post[0,0], self.X_post[1,0], self.X_post[2,0] = x, y, theta
        #     self.x_meas, self.y_meas = x, y
        #     #self.x_cov, self.y_cov = 1e-1, 1e-1
        #     #self.P[2,2] = 10e9
        #     #self.ekf_update(0.01)
        #     self.publish_odom()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.last_time_vehicle if self.last_time_vehicle > self.last_time_slam else self.last_time_slam
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        
        if self.relative_slam:
            msg.pose.pose.position = Point(self.X_post[0,0], self.X_post[1,0], 0.0)
        else:
            # Add displacement back
            msg.pose.pose.position = Point(self.X_post[0,0] + self.origin_slam_x, self.X_post[1,0] + self.origin_slam_y, 0.0)

        msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, 0.0, self.X_post[2,0]).GetQuaternion()))
        
        p_cov = np.array([0.0]*36).reshape(6,6)
        
        # position covariance
        p_cov[0:2,0:2] = self.P[0:2,0:2]
        # orientation covariance for Yaw
        # x and Yaw
        p_cov[5,0] = p_cov[0,5] = self.P[2,0]
        # y and Yaw
        p_cov[5,1] = p_cov[1,5] = self.P[2,1]
        # Yaw and Yaw
        p_cov[5,5] = self.P[2,2]
        
        msg.pose.covariance = tuple(p_cov.ravel().tolist())

        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.odom_ekf_pub.publish(msg)

        if self.relative_slam:
            msg.pose.pose.position = Point(self.x_meas, self.y_meas, 0.0)
        else:
            # Add displacement back
            msg.pose.pose.position = Point(self.x_meas + self.origin_slam_x, self.y_meas + self.origin_slam_y, 0.0)

        self.slam_odom_pub.publish(msg)

        if self.publish_odom_tf:
            self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)

    def publish(self, publish_rate):
        loop_rate = rospy.Rate(publish_rate)

        while not rospy.is_shutdown():
            if not self.filter_initialized:
                loop_rate.sleep()
                continue

            self.publish_odom()
            try:
                loop_rate.sleep()
            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn("Saw a negative time change, resetting SLAM EKF.")
                    self.filter_initialized = False

if __name__ == '__main__':
    rospy.init_node('slam_ekf_node')

    slam_ekf = RosGpsEkf()

    publish_rate = rospy.get_param('~publish_rate', 30)
    slam_ekf.slam_rate = rospy.get_param('~slam_rate', 20)

    slam_ekf.frame_id = rospy.get_param('~frame_id', '/odom')
    slam_ekf.child_frame_id = rospy.get_param('~child_frame_id', '/base_footprint')

    # Set SLAM position relative to first measurement, i.e. first measurement will be at (0,0)
    slam_ekf.relative_slam = rospy.get_param('~relative_slam', True)

    slam_ekf.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)
    
    rospy.Subscriber("/odometry/imu", Odometry, slam_ekf.slam_odom_callback, queue_size = 1)
    rospy.Subscriber("/husky_velocity_controller/odom", Odometry, slam_ekf.vehicle_state_callback, queue_size = 1)
    
    try:
        slam_ekf.publish(publish_rate)
    except rospy.ROSInterruptException:
        rospy.logdebug("Exiting")
        pass
