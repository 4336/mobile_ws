#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "pharos_myrio/MotorVelStamped.h"
#include "pharos_motion/PointArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_;
ros::Subscriber sub2_;
ros::Subscriber sub3_;
ros::Publisher pub_;
ros::Publisher pub2_;
ros::Publisher pub3_;
ros::Publisher pub_ctrl_;

pharos_motion::PointArray points_;
geometry_msgs::Point current_pos_;
geometry_msgs::Point odom_xyh_;

geometry_msgs::Vector3Stamped haptic_;
geometry_msgs::Vector3Stamped ctrlResult_;

geometry_msgs::PointStamped nextStamped_;
geometry_msgs::PointStamped nextStamped2_;


struct HapticDevice_Bits {
  bool index:1;
  bool start:1;
  bool finish:1;
};

union HapticDevice_State {
  uint8_t all;
  struct HapticDevice_Bits bit;
};

HapticDevice_State cmd_state;
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
	if(cmd_state.bit.finish) std::cout<<points_.points.size()<<" "<<nearest_index<<" "<<desired_index<<std::endl;

	next.x = points_.points[desired_index].x;
	next.y = points_.points[desired_index].y;
	if(cmd_state.bit.finish) std::cout<<p<<std::endl;

	float diff_x = next.x - p.x;
	float diff_y = next.y - p.y;
	next.z = atan2(diff_y,diff_x);

	nextStamped_.point = next;
	nextStamped_.point.z = 0;
	nextStamped_.header.stamp = ros::Time::now();
	nextStamped_.header.frame_id = "odom";

	pub2_.publish(nextStamped_);


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

	pub3_.publish(nextStamped2_);
	//
	dist = sqrt(pow(points_.points[end_idx].x-points_.points[nearest_index].x,2)
		+ pow(points_.points[end_idx].y-points_.points[nearest_index].y,2));
	if(dist < 0.2) isTrjInit = false;
	return next;
}

pharos_myrio::MotorVel FindMotorVel(geometry_msgs::Point curr, geometry_msgs::Point next)
{
	pharos_myrio::MotorVel vel;

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

	if(fabs(theta_desired) > 2*desiredVel/0.49){
		if(theta_desired > 0){
			vel.right = desiredVel;
			vel.left = -desiredVel;
		}else{
			vel.right = -desiredVel;
			vel.left = desiredVel;
		}
	}else{
		if(theta_desired > 0){
			vel.right = desiredVel;
			vel.left = desiredVel - theta_desired * 0.49;
		}else{
			vel.right = desiredVel + theta_desired * 0.49;
			vel.left = desiredVel;
		}
	}

	return vel;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
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


	static pharos_myrio::MotorVelStamped cmd;
	cmd.vel.left = 0;
	cmd.vel.right = 0;

	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "mobile_robot";

	cmd.state = 1;

	if(isTrjInit){
		geometry_msgs::Point next_xyh = FindDesiredPoint(odom_xyh_);
		pharos_myrio::MotorVel motorVel = FindMotorVel(odom_xyh_, next_xyh);

		if(cmd_state.bit.finish == 0){
			cmd.vel.left = 0;
			cmd.vel.right = 0;
		}else{
			cmd.vel = motorVel;

			// std::cout<<"next: "<<next_xyh<<std::endl;
			// std::cout<<"curr: "<<odom_xyh_<<std::endl;
		}
	}else{
		cmd.vel.left = 0;
		cmd.vel.right = 0;
	}
	pub_.publish(cmd);
}

void TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg)
{
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
}

void HapticDeviceCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	cmd_state.all = msg->vector.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_motion");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("vel", desiredVel, 0.1);
	pnh.param<float>("lookAhead", lookAhead, 0.1);

	sub_ = nh.subscribe("odom/rover", 10, OdomCallback); //topic que function
	sub2_ = nh.subscribe("trajectory", 10, TrajectoryCallback); //topic que function
	sub3_ = nh.subscribe("haptic_device", 10, HapticDeviceCallback); //topic que function
	pub_ = nh.advertise<pharos_myrio::MotorVelStamped>("motor_cmd", 10); //topic que
	pub2_ = nh.advertise<geometry_msgs::PointStamped>("look_a_head_point", 10);
	pub3_ = nh.advertise<geometry_msgs::PointStamped>("updated_path", 10);
	pub_ctrl_ = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_result", 10);

	// odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mobile_odom",1);


	ros::spin();

	return 0;
}