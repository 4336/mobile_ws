#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Bool.h"

#include "omni_msgs/OmniButtonEvent.h"
#include "std_msgs/Time.h"
#include "tf/transform_broadcaster.h"

#include "pharos_msgs/DrawvingState.h"

ros::NodeHandle *nhPtr;

//global ros msgs
ros::Subscriber clock_sub_;
ros::Subscriber button_sub_;

ros::Subscriber clear_reset_sub_;
ros::Subscriber clear_send_sub_;
ros::Subscriber clear_move_sub_;

ros::Publisher state_pub_;
//ros::Publisher pub_end_point_;


pharos_msgs::DrawvingState DrawvingState_;
pharos_msgs::DrawvingState DrawvingState_old_;


ros::Time GazeboClock_;

float scaler_;

bool isGazebo_ = true;
bool isOdomInit_ = false;
bool isStart_ = false;

ros::Time Clock()
{
	if(isGazebo_) GazeboClock_;
	return ros::Time::now();
}


void DrawvingStatePublisher(){

	DrawvingState_.header.stamp = Clock();

	switch(DrawvingState_.state){
	case DrawvingState_.RESET:
		DrawvingState_.state_str = "RESET";
		break;
	case DrawvingState_.READY:
		DrawvingState_.state_str = "READY";
		break;
	case DrawvingState_.START:
		DrawvingState_.state_str = "START";
		break;
	case DrawvingState_.SEND:
		DrawvingState_.state_str = "SEND";
		break;
	case DrawvingState_.MOVE:
		DrawvingState_.state_str = "MOVE";
		break;
	case DrawvingState_.FINISH:
		DrawvingState_.state_str = "FINISH";
		break;
	default:
		DrawvingState_.state_str = "ETC";
		break;
	}


	state_pub_.publish(DrawvingState_);
}


void ButtonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg)
{
	DrawvingState_old_ = DrawvingState_;

	DrawvingState_.header.stamp = Clock();
	DrawvingState_.index = msg->grey_button!=0;
	DrawvingState_.mode = msg->white_button!=0;

	if(DrawvingState_.mode == 0 & DrawvingState_old_.mode == 1){ // release state switch
		if(DrawvingState_.header.stamp - DrawvingState_old_.header.stamp > ros::Duration(0.5)){	// long click
			DrawvingState_.state = DrawvingState_.RESET;
		}else{ //short click
			switch(DrawvingState_.state){
				case DrawvingState_.RESET:
					DrawvingStatePublisher();
					DrawvingState_.state = DrawvingState_.READY;
					break;
				case DrawvingState_.READY:
					DrawvingState_.state = DrawvingState_.START;
					break;
				case DrawvingState_.START:
					DrawvingState_.state = DrawvingState_.SEND;
					break;
				case DrawvingState_.SEND:
					DrawvingState_.state = DrawvingState_.MOVE;
					break;
				case DrawvingState_.MOVE:
					//DrawvingState_.state = DrawvingState_.FINISH;
					break;
				case DrawvingState_.FINISH:
					DrawvingState_.state = DrawvingState_.READY;
					break;
				default:
					break;
			}
		}
	}

	DrawvingStatePublisher();
}

void ClockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	GazeboClock_ = msg->clock;
}

void ClearResetCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		DrawvingState_.state = DrawvingState_.READY;
		DrawvingStatePublisher();
	}
}

void ClearSendCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		DrawvingState_.state = DrawvingState_.MOVE;
		DrawvingStatePublisher();
	}
}

void ClearMoveCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data){
		DrawvingState_.state = DrawvingState_.FINISH;
		DrawvingStatePublisher();
	}
}

void init(){
	DrawvingState_.header.frame_id = "omni_state";
	//DrawvingState_.header.stamp = Clock();
	DrawvingState_.index = false;
	DrawvingState_.mode = false;
	DrawvingState_.state = DrawvingState_.RESET;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_omni_driver");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("scaler", scaler_, 1);
	pnh.param<bool>("gazebo", isGazebo_, true);

	clock_sub_ = nh.subscribe("/clock", 10, ClockCallback); //topic que function
	button_sub_ = nh.subscribe("/omni/phantom/button", 10, ButtonCallback); //topic que function
	clear_reset_sub_ = nh.subscribe("/omni/clear_reset", 10, ClearResetCallback); //topic que function
	clear_send_sub_ = nh.subscribe("/omni/clear_send", 10, ClearSendCallback); //topic que function
	clear_move_sub_ = nh.subscribe("/omni/clear_move", 10, ClearMoveCallback); //topic que function

	state_pub_ = nh.advertise<pharos_msgs::DrawvingState>("/master/state", 10); //topic que
	//pub_end_point_ = nh.advertise<geometry_msgs::Vector3Stamped>("/omni/point", 10); //topic que

	init();

	ros::spin();

	return 0;
}