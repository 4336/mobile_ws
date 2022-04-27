#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_odometry_;
ros::Subscriber sub_imu_;
ros::Publisher pub_odom_;

nav_msgs::Odometry odom_rover_;

geometry_msgs::Quaternion imu_ori_;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static geometry_msgs::Point p;
	static ros::Time ros_time0 = ros::Time(0);
	if(ros_time0 == ros::Time(0)){
		ros_time0 = ros::Time::now();

		p.x = 0;
		p.y = 0;
		p.z = M_PI/2.;

		odom_rover_.header.frame_id = "odom";
		odom_rover_.child_frame_id = "husky";
	}
	ros::Time ros_time = ros::Time::now();
	ros::Duration ros_dt = ros_time - ros_time0;
	float dt = ros_dt.toSec();
	// std::cout<<dt<<std::endl;

	p.x += cos(p.z) * msg->twist.twist.linear.x * dt;
	p.y += sin(p.z) * msg->twist.twist.linear.x * dt;
	p.z += msg->twist.twist.angular.z * dt;

	geometry_msgs::Quaternion msg_Q;
	msg_Q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, p.z);
	odom_rover_.pose.pose.position.x = p.x;
	odom_rover_.pose.pose.position.y = p.y;
	odom_rover_.pose.pose.orientation = msg_Q;


	odom_rover_.header.stamp = ros::Time::now();
	pub_odom_.publish(odom_rover_);
	ros_time0 = ros_time;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg){
	imu_ori_ = msg->orientation;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	sub_odometry_ = nh.subscribe("/husky_velocity_controller/odom", 10, OdometryCallback); //topic que function
	sub_imu_ = nh.subscribe("/imu/data", 10, ImuCallback); //topic que function

	pub_odom_ = nh.advertise<nav_msgs::Odometry>("odom/rover", 10); //topic que

	ros::spin();

	return 0;
}