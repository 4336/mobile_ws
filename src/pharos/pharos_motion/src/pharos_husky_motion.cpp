#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "pharos_motion/PointArray.h"
#include "pharos_msgs/DrawvingState.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_rover_odom_;
ros::Subscriber sub_trajectory_;
ros::Subscriber sub_master_state_;
ros::Subscriber sub_master_pose_;
ros::Publisher pub_cmd_vel_;
ros::Publisher pub_LAP_;
ros::Publisher pub_flowing_path_;
ros::Publisher pub_ctrl_;
ros::Publisher pub_clear_send;
ros::Publisher pub_clear_move;

pharos_motion::PointArray points_;
geometry_msgs::Point current_pos_;
geometry_msgs::Point odom_xyh_;

geometry_msgs::Point MasterPoint_;

geometry_msgs::Vector3Stamped haptic_;
geometry_msgs::Vector3Stamped ctrlResult_;

geometry_msgs::PointStamped nextStamped_;
geometry_msgs::PointStamped nextStamped2_;


pharos_msgs::DrawvingState DrawvingState_;
geometry_msgs::Point curv_;
geometry_msgs::Point curv0_;

float desiredVel;
float lookAhead;
bool isTrjInit;
int indexCnt;

int desired_index;
int nearest_index;

float curvature = 0;

geometry_msgs::Point FindDesiredPoint(geometry_msgs::Point p)
{
	static geometry_msgs::Point next;

	int end_idx = points_.points.size()-1;
	float dist;
	float dist_min = 1;
	float dist_next = 1;

	for(int i=nearest_index; i<end_idx; i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist < dist_min){
			nearest_index = i;
			dist_min = dist;
		}
	}


	for(int i=nearest_index; i<end_idx; i++){
		dist = sqrt(pow(points_.points[i].x-points_.points[nearest_index].x,2)
			+ pow(points_.points[i].y-points_.points[nearest_index].y,2));
		if(dist < lookAhead){
			desired_index = i+1;
			dist_next = dist;
		}
	}
	if(DrawvingState_.state == DrawvingState_.MOVE) std::cout<<points_.points.size()<<" "<<nearest_index<<" "<<desired_index<<std::endl;

	next.x = points_.points[desired_index].x;
	next.y = points_.points[desired_index].y;
	if(DrawvingState_.state == DrawvingState_.MOVE) std::cout<<p<<std::endl;

	float diff_x = next.x - p.x;
	float diff_y = next.y - p.y;
	next.z = atan2(diff_y,diff_x);

	nextStamped_.point = next;
	nextStamped_.point.z = 0;
	nextStamped_.header.stamp = ros::Time::now();
	nextStamped_.header.frame_id = "odom";

	pub_LAP_.publish(nextStamped_);


	curv_.x = points_.points[desired_index].x - points_.points[desired_index-1].x;
	curv_.y = points_.points[desired_index].y - points_.points[desired_index-1].y;
	curv_.z = atan2(curv_.x,curv_.y);
	curvature = (curv_.z - curv0_.z)*100;
	// std::cout<<curv_<<" "<<sqrt(pow(curv_.x,2)+pow(curv_.y,2))<<"\n";
	
	curv0_ = curv_;


	ctrlResult_.vector.x = dist_min*100.;
	// ctrlResult_.vector.y = next.z;
	// ctrlResult_.vector.z = curv_.z;
	ctrlResult_.header.stamp = ros::Time::now();
	pub_ctrl_.publish(ctrlResult_);
	std::cout<<ctrlResult_<<std::endl;

	//
	static geometry_msgs::Point next2;

	indexCnt++;
	if(indexCnt>=points_.points.size())indexCnt = 0;
	next2.x = points_.points[indexCnt].x;
	next2.y = points_.points[indexCnt].y;

	nextStamped2_.point = next2;
	nextStamped2_.point.z = 0;
	nextStamped2_.header.stamp = ros::Time::now();
	nextStamped2_.header.frame_id = "odom";

	pub_flowing_path_.publish(nextStamped2_);
	//
	dist = sqrt(pow(points_.points[end_idx].x-points_.points[nearest_index].x,2)
		+ pow(points_.points[end_idx].y-points_.points[nearest_index].y,2));

	static std_msgs::Bool clear_move_msg;
	if(dist < 0.4){
		isTrjInit = false;
		clear_move_msg.data = true;
		pub_clear_move.publish(clear_move_msg);
	}
	return next;
}

geometry_msgs::Twist FindMotorVel(geometry_msgs::Point curr, geometry_msgs::Point next)
{
	static geometry_msgs::Twist cmd_vel;

	float theta_err = next.z-curr.z;
	if(theta_err > M_PI) theta_err -= 2*M_PI;
	if(theta_err <-M_PI) theta_err += 2*M_PI;

	ctrlResult_.vector.z = theta_err/M_PI*180;
	
	static float theta_desired = 0;
	theta_desired = curvature*10. + theta_err;
	std::cout<<"output: "<<curvature<<" "<<theta_err<<std::endl;
	std::cout<<"output_real: "<<curvature*10.<<" "<<theta_err/10.<<std::endl;
	float integMax = 100;

	// if(theta_desired > integMax) theta_desired = integMax;
	// if(theta_desired <-integMax) theta_desired =-integMax;

#if 1
	if(theta_desired > M_PI) theta_desired = M_PI;
	if(theta_desired <-M_PI) theta_desired =-M_PI;
#else
	if(theta_desired > M_PI){
		do{
			theta_desired -= 2*M_PI;
		}while(theta_desired > M_PI);
	}
	if(theta_desired <-M_PI){
		do{
			theta_desired += 2*M_PI;
		}while(theta_desired <-M_PI);
	}
#endif

	std::cout<<"err/sum: "<<theta_err<<" "<<theta_desired<<"\n\n";

	cmd_vel.linear.x = desiredVel*((MasterPoint_.y+0.1)/0.15*5) - fabs(theta_desired);
	if(cmd_vel.linear.x < 0) cmd_vel.linear.x = 0;
	cmd_vel.angular.z = theta_desired;

	return cmd_vel;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static geometry_msgs::Twist cmd_vel;
	const static geometry_msgs::Twist cmd_init;
	cmd_vel = cmd_init;

	odom_xyh_.x = msg->pose.pose.position.x;
	odom_xyh_.y = msg->pose.pose.position.y;

	tf::Quaternion tf_Q;
	tf_Q.setX(msg->pose.pose.orientation.x);
	tf_Q.setY(msg->pose.pose.orientation.y);
	tf_Q.setZ(msg->pose.pose.orientation.z);
	tf_Q.setW(msg->pose.pose.orientation.w);

	geometry_msgs::Quaternion msg_Q;

	double roll, pitch, yaw;
	tf::Matrix3x3(tf_Q).getRPY(roll, pitch, yaw);
	// std::cout<<"rpy: "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
	odom_xyh_.z = yaw;



	if(isTrjInit){
		if(DrawvingState_.index){
			if(DrawvingState_.state == DrawvingState_.MOVE){
				geometry_msgs::Point next_xyh = FindDesiredPoint(odom_xyh_);
				cmd_vel = FindMotorVel(odom_xyh_, next_xyh);
			}
		}
	}
	pub_cmd_vel_.publish(cmd_vel);
}

void TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg)
{
	if(msg->poses.empty()) return;
	// clear
	points_.points.clear();
	desired_index = 1;
	nearest_index = 1;

	static tf::StampedTransform transform;
	static tf::TransformListener listener;

	// try{
	// 	listener.waitForTransform("odom", "rover", ros::Time(0), ros::Duration(1.0));
	// 	listener.lookupTransform("odom", "rover", ros::Time(0), transform);
	// }catch (tf::TransformException ex){
	// 	ROS_ERROR("%s",ex.what());
	// 	ros::Duration(0.1).sleep();
	// }

	// static geometry_msgs::Point T_point;
	// for(int i=0; i < msg->poses.back().header.seq; i++){
	// 	tf::Vector3 point_vec(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, 0);
	// 	tf::Vector3 new_point = transform*point_vec;
	// 	geometry_msgs::Point T_point;

	// 	T_point.x = new_point.getX();
	// 	T_point.y = new_point.getY();
	// 	T_point.z = 0;
	// 	points_.points.push_back(T_point);
	// }

	static geometry_msgs::Point p;
	static geometry_msgs::Point T_point;
	for(int i=0; i < msg->poses.back().header.seq; i++){
		p.x = msg->poses[i].pose.position.x;
		p.y = msg->poses[i].pose.position.y;

		T_point.x = odom_xyh_.x + p.x*cos(odom_xyh_.z) - p.y*sin(odom_xyh_.z);
		T_point.y = odom_xyh_.y + p.x*sin(odom_xyh_.z) + p.y*cos(odom_xyh_.z);

		points_.points.push_back(T_point);
	}
	// std::cout<<points_<<std::endl;
	isTrjInit = true;

	static std_msgs::Bool clear_send_msg;
	clear_send_msg.data = true;
	pub_clear_send.publish(clear_send_msg);
}

void DrawvingStateCallback(const pharos_msgs::DrawvingState::ConstPtr& msg)
{
	DrawvingState_ = *msg;
}
void MasterPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	MasterPoint_.x = -msg->pose.position.x;
	MasterPoint_.y = -msg->pose.position.y + 0.15;
	MasterPoint_.z = msg->pose.position.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_motion");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("vel", desiredVel, 0.1);
	pnh.param<float>("lookAhead", lookAhead, 0.1);

	sub_rover_odom_ = nh.subscribe("/odom/slam_ekf", 10, OdomCallback); //topic que function
	sub_trajectory_ = nh.subscribe("trajectory", 10, TrajectoryCallback); //topic que function
	sub_master_state_ = nh.subscribe("/master/state", 10, DrawvingStateCallback); //topic que function
	sub_master_pose_ = nh.subscribe("/master/phantom/pose", 10, MasterPoseCallback); //topic que function

	pub_cmd_vel_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); //topic que
	pub_LAP_ = nh.advertise<geometry_msgs::PointStamped>("look_a_head_point", 10);
	pub_flowing_path_ = nh.advertise<geometry_msgs::PointStamped>("updated_path", 10);
	pub_ctrl_ = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_result", 10);
	pub_clear_send = nh.advertise<std_msgs::Bool>("/master/clear_send", 10);
	pub_clear_move = nh.advertise<std_msgs::Bool>("/master/clear_move", 10);


	// odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mobile_odom",1);

	ros::spin();

	return 0;
}