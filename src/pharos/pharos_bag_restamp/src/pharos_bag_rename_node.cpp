#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_odom;
ros::Publisher pub_odom;

ros::Subscriber sub_pc;
ros::Publisher pub_pc;

ros::Subscriber sub_imu;
ros::Publisher pub_imu;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	pub_odom.publish(msg);
}

void PcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pub_pc.publish(msg);
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	pub_imu.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string bag = "/bag";

	std::string odom = "/odometry/filtered";
	sub_odom = nh.subscribe(odom, 10, OdomCallback); //topic que function
	pub_odom = nh.advertise<nav_msgs::Odometry>(bag + odom, 10); //topic que

	std::string pc = "/velodyne_points";
	sub_pc = nh.subscribe(pc, 10, PcCallback); //topic que function
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>(bag + pc, 10); //topic que

	std::string imu = "/imu/data";
	sub_imu = nh.subscribe(imu, 10, ImuCallback); //topic que function
	pub_imu = nh.advertise<sensor_msgs::Imu>(bag + imu, 10); //topic que

	ros::spin();

	return 0;
}