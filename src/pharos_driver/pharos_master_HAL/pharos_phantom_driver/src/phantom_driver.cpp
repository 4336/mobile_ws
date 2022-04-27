#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

#include "phantom_premium_msgs/TeleoperationDeviceStateStamped.h"

ros::NodeHandle *nhPtr;

//global ros msgs
ros::Subscriber phantom_sub_;
ros::Publisher pub_phantom_pose_;


bool isOdomInit_ = false;
bool isStart_ = false;

int publish_rate_ = 100;

float scaler_;


void PhantomCallback(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped Pose;

	Pose.header.stamp = msg->header.stamp;
	Pose.header.frame_id = "phantom";

	Pose.pose = msg->state.pose;

	pub_phantom_pose_.publish(Pose);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_phantom_driver");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("scaler", scaler_, 1);

	phantom_sub_ = nh.subscribe("/master_control/input_master_state", 1, PhantomCallback); //topic que function

	pub_phantom_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/master/pose", 10); //topic que

	pnh.param("publish_rate", publish_rate_, 100);

	ros::Rate loop_rate(publish_rate_);

	while (ros::ok())
	{
		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}