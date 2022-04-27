#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <pharos_myrio/MotorVelStamped.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub;
ros::Publisher pub_;
ros::Publisher odom_pub_;
nav_msgs::Odometry odom_rover_;


struct HapticDevice_Bits {
  bool index:1;
  bool start:1;
  bool finish:1;
};

union HapticDevice_State {
  uint8_t all;
  struct HapticDevice_Bits bit;
};

//

HapticDevice_State cmd_state;
float desiredVel;
float lookAhead;

bool isInit = false;



void MotorVelCallback(const pharos_myrio::MotorVelStamped::ConstPtr& msg)
{
	static geometry_msgs::Point p;
	if(!isInit){
		p.x = 0;
		p.y = 0;
		p.z = M_PI/2.;

		odom_rover_.header.frame_id = "odom";
		odom_rover_.child_frame_id = "rover";

		isInit = true;
	}
	float alpha = 1.1;
	float v_l = msg->vel.left * alpha;
	float v_r = msg->vel.right * alpha;
	float track = 0.49 * alpha;

	p.x += cos(p.z)*(msg->vel.left+msg->vel.right)/2*0.01;
	p.y += sin(p.z)*(msg->vel.left+msg->vel.right)/2*0.01;
	p.z += (msg->vel.right-msg->vel.left)/track*0.01;


	geometry_msgs::Quaternion msg_Q;
	msg_Q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, p.z);
	odom_rover_.pose.pose.position.x = p.x;
	odom_rover_.pose.pose.position.y = p.y;
	odom_rover_.pose.pose.orientation = msg_Q;


	odom_rover_.header.stamp = ros::Time::now();
	odom_pub_.publish(odom_rover_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	sub = nh.subscribe("motor_vel", 10, MotorVelCallback); //topic que function
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom/rover", 10); //topic que

	ros::spin();

	return 0;
}