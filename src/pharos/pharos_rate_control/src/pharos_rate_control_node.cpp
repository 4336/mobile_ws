#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "pharos_myrio/MotorVelStamped.h"

#include "math.h"

struct HapticDevice_Bits {
	bool index:1;
	bool start:1;
	bool finish:1;
};

union HapticDevice_State {
	uint8_t all;
	struct HapticDevice_Bits bit;
};

HapticDevice_State state_;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_;
ros::Publisher pub_;
ros::Publisher pub2_;

geometry_msgs::Point pos;



void RateCtrlMotorControll()
{
	static pharos_myrio::MotorVelStamped motorCmd;

	motorCmd.state = state_.bit.index;

	static float scale;

	if(state_.bit.index) scale = 5;
	else scale = 0;

	motorCmd.vel.left = pos.y * 0.001 * scale;
	motorCmd.vel.right = pos.y * 0.001 * scale;

	motorCmd.header.stamp = ros::Time::now();
	motorCmd.header.frame_id = "haptic_device";

	pub_.publish(motorCmd);
}

void RateCtrlForceFeedback()
{
	static geometry_msgs::Vector3Stamped forceCmd;
	static float radius = 10;
	static float kp_ = 0.03;
	float norm = sqrt(pos.x*pos.x + pos.y*pos.y);

	float theta = atan2(pos.y, pos.x);
	if(norm > radius){
		forceCmd.vector.x = - (norm - radius) * cos(theta) * kp_;
		forceCmd.vector.y = - (norm - radius) * sin(theta) * kp_;
	}else{
		forceCmd.vector.x = 0;
		forceCmd.vector.y = 0;
	}
	forceCmd.vector.z = state_.bit.index;

	forceCmd.header.stamp = ros::Time::now();
	forceCmd.header.frame_id = "haptic_device";

	pub2_.publish(forceCmd);
}

void Callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	pos.x = msg->vector.x;
	pos.y = msg->vector.y;
	state_.all = msg->vector.z;
	
	RateCtrlMotorControll();
	RateCtrlForceFeedback();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	sub_ = nh.subscribe("sub", 10, Callback); //topic que function
	pub_ = nh.advertise<pharos_myrio::MotorVelStamped>("pub", 10); //topic que
	pub2_ = nh.advertise<geometry_msgs::Vector3Stamped>("pub2", 10); //topic que

	ros::spin();

	return 0;
}