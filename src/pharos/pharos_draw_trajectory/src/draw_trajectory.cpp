#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "pharos_msgs/DrawvingState.h"
#include "std_msgs/Time.h"
#include "std_msgs/Bool.h"
#include "tf/transform_broadcaster.h"
#define B_Spline_Order 5
#define B_Spline_Dt 1
#include "b_spline.h"


//global ros msgs
ros::Subscriber odom2tf_sub_;
ros::Subscriber sub_master_pose_;
ros::Subscriber drawving_state_sub_;
ros::Subscriber sub_rover_odom_;
// ros::Publisher pub_start_odom_;
ros::Publisher drawing_pub_;
ros::Publisher trajectory_pub_;
ros::Publisher pub_end_point_;
ros::Publisher pub_master_clear_reset;

geometry_msgs::Point pos_;
geometry_msgs::Point pos_old_;
nav_msgs::Path path_;
nav_msgs::Path traj_;
nav_msgs::Odometry start_odom_;
nav_msgs::Odometry rover_odom_;

geometry_msgs::Vector3Stamped end_point_;


//global custom structures
pharos_msgs::DrawvingState State_;
pharos_msgs::DrawvingState State_old_;

float scaler_;
std::string frame_id_;

bool isGazebo_ = true;
bool isForward_ = true;
bool isOdomInit_ = false;
bool clear_request_ = true;


void Init(){
	path_.header.frame_id = frame_id_;

	start_odom_.pose.pose.orientation.w = 1;
}


void Odometry2TF(tf::TransformBroadcaster &br, const nav_msgs::OdometryConstPtr &Odom){

	tf::Transform transform;
	transform.setOrigin( tf::Vector3(Odom->pose.pose.position.x, Odom->pose.pose.position.y, Odom->pose.pose.position.z) );

	tf::Quaternion quat;
	quat.setX(Odom->pose.pose.orientation.x);
	quat.setY(Odom->pose.pose.orientation.y);
	quat.setZ(Odom->pose.pose.orientation.z);
	quat.setW(Odom->pose.pose.orientation.w);
	transform.setRotation(quat);

	br.sendTransform(tf::StampedTransform(transform, Odom->header.stamp, Odom->header.frame_id, Odom->child_frame_id));
	// std::cout<<Odom->child_frame_id<<std::endl;
}

void UpdatePath(int *seq)
{
	static ros::Time stamp;
	static geometry_msgs::PoseStamped Pos;

	if(clear_request_){
		*seq = 0;
		path_.header.stamp = ros::Time::now();
		Pos.pose.position.x = 0;
		Pos.pose.position.y = 0;
		Pos.pose.position.z = 0;

		Pos.pose.orientation.x = 0;
		Pos.pose.orientation.y = 0;
		Pos.pose.orientation.z = 0;
		Pos.pose.orientation.w = 1;
		clear_request_ = false;
	}

	Pos.pose.position.y -= pos_.x - pos_old_.x;
	Pos.pose.position.x += pos_.y - pos_old_.y;

	Pos.header.seq = (*seq)++;
	Pos.header.stamp = ros::Time::now();

	path_.poses.push_back(Pos);

	end_point_.header.stamp = ros::Time::now();
	end_point_.header.frame_id = frame_id_;

	end_point_.vector.x = Pos.pose.position.x;
	end_point_.vector.y = Pos.pose.position.y;
	end_point_.vector.z = pos_.z;

	pub_end_point_.publish(end_point_);
}



void DrawvingStateCallback(const pharos_msgs::DrawvingState::ConstPtr& msg)
{
	static std_msgs::Bool clear_reset;
	clear_reset.data = true;

	State_old_ = State_;

	State_ = *msg;

	if(State_.state == State_.RESET){
	}

	if(State_.index == false & State_.mode == false){
		switch(State_.state){
			case State_.RESET:
				path_.poses.clear();
				traj_.poses.clear();
				trajectory_pub_.publish(traj_);
				clear_request_ = true;
				isForward_ = true;

				pub_master_clear_reset.publish(clear_reset);
				ROS_WARN("Path Reset Request Clear");
				break;
			case State_.READY:
				if(!State_.index & State_old_.index){
					static ros::Time Tic = ros::Time(0);
					if(Tic == ros::Time(0)){
						Tic = ros::Time::now();
					}else if(ros::Time::now() - Tic > ros::Duration(1)){
						Tic = ros::Time::now();
					}else{
						if(ros::Time::now() - Tic < ros::Duration(0.3)){
							isForward_ = !isForward_;
						}
						Tic = ros::Time(0);

						if(isForward_){
							ROS_WARN("Forward Mode");
						}else{
							ROS_WARN("Backward Mode");
						}
					}
				}
				break;
				
			case State_.START:
				if(State_old_.state != State_.START) ROS_WARN("Drawing Start");
				break;

			case State_.SEND:
				if(State_.mode != State_old_.mode){
					traj_ = path_;
					path_.poses.clear();
					clear_request_ = true;
				}
				drawing_pub_.publish(path_);
				trajectory_pub_.publish(traj_);

				if(State_old_.state != State_.SEND) ROS_WARN("Trajecoty Sending...");
				break;

			case State_.MOVE:
				if(State_old_.state != State_.MOVE) ROS_WARN("Speed Control Mode");
				break;

			default:
				break;
		}
	}
}

void StatePoseCallback(tf::TransformBroadcaster &br, const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	static int seq = 0;
	static geometry_msgs::Point stylus;
	stylus = msg->pose.position;

	pos_.x = stylus.x *1 * scaler_;
	pos_.y = stylus.z *-1 * scaler_;
	pos_.z = stylus.y *1 ;
	// state_.all = (int)msg->vector.z;

	
	if(State_.index){
		if(State_.state == State_.START){
			UpdatePath(&seq);
			path_.header.stamp = ros::Time::now();
			drawing_pub_.publish(path_);
		}
	}


// START odom to start_point TF Publisher

	start_odom_.header.stamp = ros::Time::now();
	// pub_start_odom_.publish(start_odom_);

	static nav_msgs::Odometry odom_old;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(start_odom_.pose.pose.position.x, start_odom_.pose.pose.position.y, start_odom_.pose.pose.position.z) );

	tf::Quaternion quat;
	quat.setX(start_odom_.pose.pose.orientation.x);
	quat.setY(start_odom_.pose.pose.orientation.y);
	quat.setZ(start_odom_.pose.pose.orientation.z);
	quat.setW(start_odom_.pose.pose.orientation.w);
	transform.setRotation(quat);

	if(odom_old.pose.pose.position.x != start_odom_.pose.pose.position.x || odom_old.pose.pose.position.y != start_odom_.pose.pose.position.y){
		br.sendTransform(tf::StampedTransform(transform, start_odom_.header.stamp, start_odom_.header.frame_id, start_odom_.child_frame_id));
	}else{
		// ROS_WARN("Same Transform");
	}

	odom_old = start_odom_;

// odom to start_point TF Publisher END

	pos_old_ = pos_;
}

void RoverOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	rover_odom_ = *msg;
	rover_odom_.child_frame_id = frame_id_;


	if(!isOdomInit_){
		start_odom_.header.frame_id = "odom";
		start_odom_.child_frame_id = "start_point";
		isOdomInit_ = true;
	}
	start_odom_.pose = rover_odom_.pose;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_draw_trajectory");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("scaler", scaler_, 1);
	pnh.param<bool>("gazebo",isGazebo_, false);
	pnh.param<std::string>("frame_id", frame_id_, "start_point");

	static tf::TransformBroadcaster br;
	// odom2tf_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom/start_point", 10, 
	// 	boost::bind(&Odometry2TF, boost::ref(br), _1));
	// sub_master_pose_ = nh.subscribe("/master/pose", 10, StatePoseCallback); //topic que function
	sub_master_pose_ = nh.subscribe<geometry_msgs::PoseStamped>("/master/pose", 10, 
		boost::bind(&StatePoseCallback, boost::ref(br), _1));
	drawving_state_sub_ = nh.subscribe("/master/state", 10, DrawvingStateCallback); //topic que function
	sub_rover_odom_ = nh.subscribe("/odom/vehicle", 10, RoverOdomCallback); //topic que function

	// pub_start_odom_ = nh.advertise<nav_msgs::Odometry>("/odom/start_point", 10); //topic que
	drawing_pub_ = nh.advertise<nav_msgs::Path>("/drawing", 10); //topic que
	trajectory_pub_ = nh.advertise<nav_msgs::Path>("/trajectory", 10); //topic que
	pub_end_point_ = nh.advertise<geometry_msgs::Vector3Stamped>("/master/end_point", 10); //topic que
	pub_master_clear_reset = nh.advertise<std_msgs::Bool>("/master/clear_reset", 10);

	
	Init();

	ros::spin();

	return 0;
}