#!/usr/bin/env python
import numpy as np
from math import *

import rospy
import roslib
import tf
import PyKDL as kdl
# Messages
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Twist
from tf.transformations import euler_from_quaternion

from tf import transformations
from std_msgs.msg import Int32

import copy
import math
import time

def normalizeHeading(heading):
	if heading > math.pi:
		heading -= 2*math.pi
	elif heading < -math.pi:
		heading += 2*math.pi
	return heading

class RosGpsEkf:
	"""docstring for RosGpsEkf"""
	slam_ekf_odom_pub = rospy.Publisher('/odom/slam_ekf', Odometry, queue_size=1)

	relative_slam = False
	publish_odom_tf = True

	frame_id = ''
	child_frame_id = ''

	shift = 0


	def __init__(self):
		self.ekf_reset()

	def ekf_reset(self):
		print('enter reset')
		# EKF variables
		self.stop_index = True
		self.heading_init = False
		self.Q = np.mat(np.diag([0.01, 0.01, 100, 100]))
		self.Q_init = copy.deepcopy(self.Q)
		self.P = np.mat(np.diag([0.5, 0.5, 10, 0]))
		# self.P = np.mat(np.diag([0, 0, 0]))
		self.R = np.mat(np.diag([100, 100, 0.01, 0.01]))
		self.R_init = copy.deepcopy(self.R)

		# self.R = np.mat(np.diag([2, 2, 3]))

		self.F = np.mat(np.diag([1.0, 1.0, 1.0, 1.0]))
		self.H = np.mat(np.diag([1, 1, 1, 0]))
		self.X = np.zeros((4, 1))
		self.old_X = np.mat('0.0; 0.0; 0.0; 0.0')

	# initialize variables
		self.x_meas, self.y_meas, self.theta_meas = [0.0] * 3
		# self.stop_x, self.stop_y, self.x_bias, self.y_bias = [0.0] * 4
		self.last_time = rospy.get_rostime()
		self.filter_initialized = False
		self.old_slam = np.mat([0, 0, 0, 0]).T
		self.new_slam = np.mat([0, 0, 0, 0]).T
		# self.particle_odom = [0.0, 0.0, 0.0]
		self.old_vel = 0.0
		self.old_alpha = 0.0
		self.back = False
		# self.restart = False
		# self.delta = 0
		self.inclination = 0
		# self.current_wp_index = 1000

		self.origin_slam_x = 0.0
		self.origin_slam_y = 0.0

		# self.lane_offset = 0.0
		# self.heading_difference = 0.0
		# self.lateral_offset = 0.0
		# self.curvature = 0.0

		self.covariance = np.zeros(16)


		self.steer_bias = 15.0
		self.azimuth = 0.0

		self.Y = []
		self.vscount = 0
		self.time4bagplay = 0
		self.stear_const = 15.14
		self.alpha_gain = 0.95
		self.vel_gain = 1.02
		self.cov_a = 0.2

		self.last_time_state = rospy.get_rostime()
		self.state = Point(0,0,0);
		self.state_old = Point(0,0,0);

		self.heading = 0.0;
		self.heading_old = 0.0;
		self.impact_cnt = 0;

		self.pos_old = np.array([0.0, 0.0])
		self.pos = np.array([0.0, 0.0])

		print('init')

	def husky_model(self, x, y, theta, z, vel, alpha, dt):

		d = vel*self.vel_gain * dt
		delta = self.inclination
		# gamma = d / L / cos(alpha)
		x = x + d * cos(theta)*cos(delta)
		y = y + d * sin(theta)*cos(delta)
		z = z + d * sin(delta)

		theta = theta + alpha*dt

		x_dot = - d * sin(theta) * cos(delta)
		y_dot = d * cos(theta) * cos(delta)


		return x, y, theta, z, x_dot, y_dot

	def ekf_init(self, init_x, init_y, init_theta, init_z):
		self.X = np.mat([init_x, init_y, init_theta, init_z]).T

	def ekf_predict(self, new_vel, new_alpha, dt):
		x, y, theta, z, x_dot, y_dot = self.husky_model(self.X[0,0], self.X[1,0], self.X[2,0], self.X[3,0],
													   self.old_vel, self.old_alpha, dt)

	# model linearization
		self.F[0, 2] = x_dot
		self.F[1, 2] = y_dot

		self.P = self.F * self.P * self.F.T + self.Q
		K = self.P * self.H.T * (self.H * self.P * self.H.T + self.R).I

		if new_vel < 0.5:
			if abs(self.old_slam[2,0] - self.new_slam[2,0]) > 0.5:
				self.Q[2, 2] = 0
		if new_vel == 0 :
			self.Q[0,0] = 0; self.Q[1,1] = 0
		else:
			self.stop_index = True

		self.P = (np.eye(4) - K * self.H) * self.P

		self.X = np.mat([x, y, theta, z]).T

		return K

	# SLAM update
	def ekf_update(self, x_meas, y_meas, theta_meas, z_meas, K):
		if theta_meas > 0 and self.X[2,0] < 0 and theta_meas > 1.5:
			theta_meas-=math.pi*2
		elif theta_meas < 0 and self.X[2,0] > 0 and theta_meas <-1.5:
			theta_meas+=math.pi*2

		if self.back == True:
			if theta_meas < 0:
				theta_meas += math.pi
			elif theta_meas > 0:
				theta_meas -= math.pi

		Z = np.mat([x_meas, y_meas, theta_meas, z_meas]).T
		S = Z - self.X
		S[2, 0] = normalizeHeading(S[2,0])
		self.X = self.X + K * S
		self.X[2,0] = normalizeHeading(self.X[2,0])



	def slam_odom_callback(self, msg):


		self.vscount = 0;

		self.covariance = msg.pose.covariance

		euler = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
															msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))


		if abs(msg.pose.pose.position.x - self.X[0])>5:
			self.impact_cnt = self.impact_cnt + 1
			if self.impact_cnt < 10:
				rospy.logwarn(self.impact_cnt)
				return
			else:
				self.ekf_reset()
				init_pos = np.mat([msg.pose.pose.position.x,
					msg.pose.pose.position.y,
					euler[2],
					msg.pose.pose.position.z]).T
				self.ekf_init(init_pos[0,0], init_pos[1,0], init_pos[2,0], init_pos[3,0])
				rospy.logwarn("SLAM INIT")

				self.impact_cnt = 0

		rover = msg

		rover.pose.pose.position.x = msg.pose.pose.position.x + self.shift*cos(euler[2])
		rover.pose.pose.position.y = msg.pose.pose.position.y + self.shift*sin(euler[2])

		

		if not self.filter_initialized:
			self.ekf_reset()

			# We always calculate in relative coordinates, but then displace result to global frame
			self.origin_slam_x = rover.pose.pose.position.x
			self.origin_slam_y = rover.pose.pose.position.y
			self.origin_altitude = rover.pose.pose.position.z
			rover.pose.pose.position.z = 0
			# first calculation of relative pos measurement
			new_slam = np.mat([rover.pose.pose.position.x,
							  rover.pose.pose.position.y,
							  euler[2],
							  rover.pose.pose.position.z]).T

			self.ekf_init(new_slam[0,0], new_slam[1,0], new_slam[2,0], new_slam[3,0])

			if self.relative_slam:
				print 'origin'
				rospy.set_param('~origin/x', self.origin_slam_x)
				rospy.set_param('~origin/y', self.origin_slam_y)
				rospy.loginfo("Using relative SLAM coordinates. Origin stored in ~/origin")

			self.filter_initialized = True
			self.last_time = rospy.get_rostime()
		else:
			new_slam = np.mat([rover.pose.pose.position.x,
							  rover.pose.pose.position.y,
							  euler[2],
							  rover.pose.pose.position.z]).T

			# predict_vect, predict_time = self.slam_predict(msg.header.stamp)

			# predicted_stamp = msg.header.stamp + predict_time

			# x = predict_vect[0]
			# y = predict_vect[1]
			# new_slam[0] += sqrt(x*x+y*y)*cos(atan2(y, x) + new_slam[2])
			# new_slam[1] += sqrt(x*x+y*y)*sin(atan2(y, x) + new_slam[2])
			# new_slam[2] += predict_vect[2]

			# covx=self.covariance[0]/0.05
			# covy=self.covariance[7]/0.05
			# covt=self.covariance[35]/0.0005

			covx=0.5
			covy=0.5
			covt=0.01

			self.Q[0,0]=self.Q_init[0,0]*(1-self.cov_a)+self.Q_init[0,0]*self.cov_a/covx
			self.Q[1,1]=self.Q_init[1,1]*(1-self.cov_a)+self.Q_init[1,1]*self.cov_a/covy
			self.Q[2,2]=self.Q_init[2,2]*(1-self.cov_a)+self.Q_init[2,2]*self.cov_a/covt
			self.R[0,0]=self.R_init[0,0]*(1-self.cov_a)+self.R_init[0,0]*self.cov_a*covx
			self.R[1,1]=self.R_init[1,1]*(1-self.cov_a)+self.R_init[1,1]*self.cov_a*covy
			self.R[2,2]=self.R_init[2,2]*(1-self.cov_a)+self.R_init[2,2]*self.cov_a*covt


			if self.time_state:
				dt = (self.time4bagplay-self.last_time).to_sec()
			else:
				dt = (rospy.Time.now()-self.last_time).to_sec()

			if dt < 0 or dt > 0.1:
				dt = 0.01;

			K = self.ekf_predict(self.old_vel, self.old_alpha, dt)


				#############################################################################################################################
			if abs(self.old_slam[2,0] - self.new_slam[2,0])< 0.5:# and self.covariance[0] < 15:
				if not self.old_vel == 0:
					self.ekf_update(new_slam[0,0], new_slam[1,0], new_slam[2,0], new_slam[3,0], K)
				#############################################################################################################################


			if self.time_state:
				self.last_time = self.time4bagplay
			else:
				self.last_time = rospy.Time.now()

		if abs(self.old_X[0] - self.X[0])>5:
			rospy.logwarn("slam pose jump")
		self.old_X = self.X

		self.publish_odom()

		self.old_slam = self.new_slam
		self.new_slam = new_slam



	def vehicle_state_callback(self, msg):
		self.time4bagplay = msg.header.stamp


		self.old_state = self.state;
		self.state = msg.pose.pose.position;

		global roll, pitch
		self.heading_old = self.heading;
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, self.heading) = euler_from_quaternion (orientation_list)

		###
		
		slam_dt = (msg.header.stamp - self.last_time_state).to_sec()

		dt = (msg.header.stamp - self.last_time_state).to_sec()
		self.last_time_state = msg.header.stamp;


		self.pos_old = self.pos
		self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

		isBack =  np.dot(np.array([cos(self.heading),sin(self.heading)]), self.pos - self.pos_old) < 0
		# if isBack:
		# 	self.back = False
		# else:
		# 	self.back = True


		new_vel = sqrt((self.state.x - self.old_state.x)**2 + (self.state.y - self.old_state.y)**2)/dt

		if isBack:
			new_vel = -new_vel

		new_alpha = atan2(sin(self.heading-self.heading_old), cos(self.heading-self.heading_old))/dt


		if not self.filter_initialized:
			return

		# R meas
		if self.heading_init == True:
			if abs(new_vel) <= 0.5:
				self.R = np.mat(np.diag([-513.8 * new_vel * new_vel + 1000, -513.8 * new_vel * new_vel + 1000,
										 -508.6 * new_vel * new_vel + 1000, 100]))
				self.Q = np.mat(np.diag([0.0, 0.0, 0.0, 0.0]))
			else:
				self.R = copy.deepcopy(self.R_init)
				self.Q = copy.deepcopy(self.Q_init)
			# for covariance
		covariance_x = self.covariance[0];
		covariance_y = self.covariance[7];
		covariance_theta = self.covariance[14]
		if covariance_x > 8 or covariance_y > 8:
			self.Q[0,0] = 0.2; self.Q[1,1] = 0.2; self.Q[2,2] = 0.01
			self.R[0,0] = pow(covariance_x-8,2)+15.0
			self.R[1,1] = pow(covariance_y-8,2)+15.0
			self.R[2,2] = pow(covariance_theta-5,2)+10.0
		else:
			self.Q = copy.deepcopy(self.Q_init)           

		# Heading initializing
		# if self.current_wp_index < 570:
		#     self.heading_init = True
		if self.heading_init == False:
			if new_vel < 1.388:
				self.Q[2, 2] = 100
			else:
				self.heading_init = True
		else:
			self.Q = copy.deepcopy(self.Q_init)

		if self.time_state:
			dt = (self.time4bagplay-self.last_time).to_sec()
		else:
			dt = (rospy.Time.now()-self.last_time).to_sec()

		if dt < 0 or dt > 0.1:
			dt = 0.01

		K = self.ekf_predict(new_vel, new_alpha, dt)

		
		if abs(self.old_X[0] - self.X[0])>5:
			rospy.logwarn("vehicle pose jump")
		self.old_X = self.X

		self.publish_odom()

		self.old_vel = new_vel
		self.old_alpha = new_alpha

		if self.time_state:
			self.last_time = self.time4bagplay
		else:
			self.last_time = rospy.Time.now()


	def publish_odom(self):
		msg = Odometry()
		msg.header.stamp = self.last_time
		msg.header.frame_id = self.frame_id
		msg.child_frame_id = self.child_frame_id

		ekf_heading = self.X[2,0]
		if self.relative_slam:
			msg.pose.pose.position = Point(self.X[0,0], self.X[1,0], self.X[3,0])
		else:
			msg.pose.pose.position = Point(self.X[0,0] + self.origin_slam_x, self.X[1,0] + self.origin_slam_y, 0.0)

		msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, -self.inclination, ekf_heading).GetQuaternion()))
		p_cov = np.array([0.0] * 36).reshape(6, 6)
	# position covariance
		p_cov[0:2, 0:2] = self.P[0:2, 0:2]
	# x and Yaw
		p_cov[5, 0] = p_cov[0, 5] = self.P[2, 0]
	# y and Yaw
		p_cov[5, 1] = p_cov[1, 5] = self.P[2, 1]
	# Yaw and Yaw
		p_cov[5, 5] = self.P[2, 2]

		msg.pose.covariance = tuple(p_cov.ravel().tolist())
		# pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
		# ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

		self.slam_ekf_odom_pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('slam_ekf_node')

	slam_ekf = RosGpsEkf()


	slam_ekf.time_state = rospy.get_param('~bagfile', False)
	slam_ekf.frame_id = rospy.get_param('~frame_id', '/odom')
	slam_ekf.child_frame_id = rospy.get_param('~child_frame_id', '/odom_ekf')

	# Set SLAM position relative to first measurement, i.e. first measurement will be at (0,0)
	slam_ekf.relative_slam = rospy.get_param('~relative_slam', True)
	slam_ekf.shift = rospy.get_param('~shift', -0.55)
	# slam_ekf.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)

	rospy.Subscriber("/odometry/imu", Odometry, slam_ekf.slam_odom_callback, queue_size = 1)
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, slam_ekf.vehicle_state_callback, queue_size = 1)

	# rospy.Subscriber('/vehicle/motion_command', MotionCommandStamped3, slam_ekf.motion_command_callback, queue_size=1)

	rospy.spin()