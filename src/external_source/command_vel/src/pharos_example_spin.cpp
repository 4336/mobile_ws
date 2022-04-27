#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub;
ros::Publisher pub;

void Callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	std::cout<<msg<<std::endl;
	pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	sub = nh.subscribe("test_sub", 10, Callback); //topic que function
	pub = nh.advertise<geometry_msgs::Vector3Stamped>("test_pub", 10); //topic que

	ros::spin();

	return 0;
}