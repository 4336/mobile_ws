#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3Stamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub;
ros::Publisher pub;

void Callback(const nav_msgs::Odometry odom)
{
	std::cout<<"1"<<std::endl;

	static geometry_msgs::Twist HuskyVel;
	float odom_x_past, odom_y_past, odom_z_past;
	
	HuskyVel.linear.x = odom.pose.pose.position.x - odom_x_past;
    HuskyVel.linear.y = odom.pose.pose.position.y - odom_y_past;
    HuskyVel.linear.z = odom.pose.pose.position.z - odom_z_past;
    HuskyVel.angular.x = 0;
    HuskyVel.angular.y = 0;
    HuskyVel.angular.z = 0;

    odom_x_past = odom.pose.pose.position.x;
    odom_y_past = odom.pose.pose.position.y;
    odom_z_past = odom.pose.pose.position.z;


	pub.publish(HuskyVel);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_vel");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::cout<<"0"<<std::endl;	
	sub = nh.subscribe("odom/stylus", 10, Callback); //topic que function
	std::cout<<"2"<<std::endl;	
	pub = nh.advertise<geometry_msgs::Twist>("test_pub", 10); //topic que

	ros::spin();

	return 0;
}