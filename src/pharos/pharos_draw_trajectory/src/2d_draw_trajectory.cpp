#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"

struct HapticDevice_Bits {
	bool index:1;
	bool start:1;
	bool finish:1;
};

union HapticDevice_State {
	uint8_t all;
	struct HapticDevice_Bits bit;
};

//global ros msgs
ros::Subscriber sub_;
ros::Subscriber odom_sub_;
ros::Publisher odom_pub_;
ros::Publisher pub_;
ros::Publisher pub2_;
ros::Publisher pub_end_point_;

geometry_msgs::Point pos_;
geometry_msgs::Point pos_old_;
nav_msgs::Path path_;
nav_msgs::Odometry start_odom_;
nav_msgs::Odometry last_odom_;

geometry_msgs::Vector3Stamped end_point_;

//global custom structures
HapticDevice_State state_;
HapticDevice_State state_old_;

float scaler_;
std::string frame_id_;

bool isOdomInit_ = false;

void Init(){
	path_.header.frame_id = frame_id_;
}

void UpdatePath(int *seq)
{
	static ros::Time stamp;
	static geometry_msgs::PoseStamped Pos;

	if(*seq == 0){
		path_.header.stamp = ros::Time::now();
		Pos.pose.position.x = 0;
		Pos.pose.position.y = 0;
		Pos.pose.position.z = 0;

		Pos.pose.orientation.x = 0;
		Pos.pose.orientation.y = 0;
		Pos.pose.orientation.z = 0;
		Pos.pose.orientation.w = 1;
	}

	Pos.pose.position.y -= pos_.x - pos_old_.x;
	Pos.pose.position.x += pos_.y - pos_old_.y;

	Pos.header.seq = (*seq)++;
	Pos.header.stamp = ros::Time::now();

	path_.poses.push_back(Pos);

	end_point_.header.stamp = ros::Time::now();
	end_point_.header.frame_id = "start_point";

	end_point_.vector.x = Pos.pose.position.x;
	end_point_.vector.y = Pos.pose.position.y;

	pub_end_point_.publish(end_point_);
}


void Callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	static geometry_msgs::PoseStamped Pos;
	static geometry_msgs::Point start_point;

	static bool isStart = false;
	static bool isWarned = false;
	static int seq = 0;

	pos_.x = msg->vector.x *0.001 * scaler_;
	pos_.y = msg->vector.y *0.001 * scaler_;
	state_.all = (int)msg->vector.z;

	if(state_.bit.index){
		UpdatePath(&seq);
		if(!isStart && !isWarned){
			ROS_WARN("please press start button first");
			isWarned = true;
		}
		path_.header.stamp = ros::Time::now();
		path_.header.frame_id = "rover";
		pub_.publish(path_);
	}

	// if(!state_.bit.index){	//capture falling edge
	// 	start_point = pos_;
	// }

//Drawing Start!
	if(state_old_.bit.start && !state_.bit.start){	//capture falling edge
		isStart = true;
		isWarned = false;
		seq = 0;
		path_.poses.clear();
		start_odom_ = last_odom_;
		isOdomInit_ = true;
	}

//Drawing Finish!
	if(!state_old_.bit.finish && state_.bit.finish){	//capture rising edge
		if(isStart){
			path_.header.stamp = ros::Time::now();
			path_.header.frame_id = frame_id_;
			pub2_.publish(path_);
		}
		path_.poses.clear();
		path_.header.frame_id = "rover";
		pub_.publish(path_);
		seq = 0;
		isStart = false;
		isWarned = false;
	}
	if(isOdomInit_){
		start_odom_.header.stamp = ros::Time::now();
		odom_pub_.publish(start_odom_);
	}

	state_old_ = state_;
	pos_old_ = pos_;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	last_odom_ = *msg;
	last_odom_.child_frame_id = frame_id_;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("scaler", scaler_, 1);
	pnh.param<std::string>("frame_id", frame_id_, "start_point");

	sub_ = nh.subscribe("haptic_device", 10, Callback); //topic que function
	odom_sub_ = nh.subscribe("odom/rover", 10, OdomCallback); //topic que function
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom/start_point", 10); //topic que
	pub_ = nh.advertise<nav_msgs::Path>("drawing", 10); //topic que
	pub2_ = nh.advertise<nav_msgs::Path>("trajectory", 10); //topic que
	pub_end_point_ = nh.advertise<geometry_msgs::Vector3Stamped>("/haptic/end_point", 10); //topic que
	
	Init();

	ros::spin();

	return 0;
}