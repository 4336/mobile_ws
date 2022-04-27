#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pharos_myrio/MotorVelStamped.h"
#include "pharos_motion/PointArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_;
ros::Subscriber sub2_;
ros::Publisher pub_;
ros::Publisher pub2_;
ros::Publisher odom_pub_;

pharos_motion::PointArray points_;
geometry_msgs::Point current_pos_;
nav_msgs::Odometry mobile_odom_;

bool isInit = false;
float desiredVel;

geometry_msgs::Point FindCurrentPosition(pharos_myrio::MotorVel vel, bool isInit)
{
	static geometry_msgs::Point p;

	if(!isInit){
		p.x = 0;
		p.y = 0;
		p.z = 0;
	}

	p.x += sin(p.z)*(vel.left+vel.right)/2*0.01;
	p.y += cos(p.z)*(vel.left+vel.right)/2*0.01;
	p.z += (vel.left-vel.right)/0.49*0.01;

	mobile_odom_.header.stamp = ros::Time::now();
	mobile_odom_.header.frame_id = "odom";
	mobile_odom_.child_frame_id = "mobile_odom";

	mobile_odom_.pose.pose.position.x = p.x;
	mobile_odom_.pose.pose.position.y = p.y;
	mobile_odom_.pose.pose.position.z = 0;

	// tf::Quaternion tf_Q;
	// geometry_msgs::Quaternion msg_Q;
	// tf_Q.setRPY(0, 0, p.z);
	// tf2::convert(msg_Q, tf_Q);
	mobile_odom_.pose.pose.orientation.x = 0;
	mobile_odom_.pose.pose.orientation.y = 0;
	mobile_odom_.pose.pose.orientation.z = 0;
	mobile_odom_.pose.pose.orientation.w = 1;

	odom_pub_.publish(mobile_odom_);

	return p;
}

geometry_msgs::Point FindDesiredPoint(geometry_msgs::Point p)
{
	static geometry_msgs::Point diff;
	float dist;
	float dist_min = 1;
	float dist_next = 1;
	int index;
	for(int i=0; i<points_.points.size(); i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist < dist_min){
			index = i;
			dist_min = dist;
		}
	}
	for(int i=index; i<points_.points.size(); i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist > 0.1){
			index = i;
			dist_next = dist;
		}
	}
	diff.x = points_.points[index].x - p.x;
	diff.y = points_.points[index].y - p.y;
	diff.z = atan2(diff.x, diff.y);
	return diff;
}

pharos_myrio::MotorVel FindMotorVel(geometry_msgs::Point curr, geometry_msgs::Point next)
{
	pharos_myrio::MotorVel vel;

	std::cout<<"next.z: "<<next.z<<std::endl;
	std::cout<<"curr.z: "<<curr.z<<std::endl;
	float theta_dot = (next.z-curr.z)/0.01;

	std::cout<<"theta_dot: "<<theta_dot<<std::endl;
	if(theta_dot > 0){
		vel.right = desiredVel - theta_dot * 0.49;
		vel.left = desiredVel;
	}else{
		vel.right = desiredVel;
		vel.left = desiredVel - theta_dot * 0.49;
	}

	return vel;
}

void MotorVelCallback(const pharos_myrio::MotorVelStamped::ConstPtr& msg)
{
	std::cout<<"MotorVelCallback start\n";
	static pharos_myrio::MotorVelStamped cmd;
	cmd.vel.left = 0;
	cmd.vel.right = 0;

	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "mobile_robot";

	cmd.state = 1;

	std::cout<<"FindCurrentPosition start\n";
	geometry_msgs::Point currentPoint = FindCurrentPosition(msg->vel, isInit);

	if(isInit){
		std::cout<<"FindDesiredPoint start\n";
		geometry_msgs::Point nextPoint = FindDesiredPoint(currentPoint);
		std::cout<<"FindMotorVel start\n";
		pharos_myrio::MotorVel motorVel = FindMotorVel(currentPoint, nextPoint);
		std::cout<<"FindMotorVel end\n";

		cmd.vel.left = motorVel.left;
		cmd.vel.right = motorVel.right;

		pub_.publish(cmd);
	}
	std::cout<<"MotorVelCallback end\n";
}

void TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg)
{
	isInit = true;

	static geometry_msgs::Point point;

	for(int i=0; i < msg->poses.back().header.seq; i++){
		point.x = msg->poses[i].pose.position.x;
		point.y = msg->poses[i].pose.position.y;
		points_.points.push_back(point);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_motion");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("vel", desiredVel, 0.1);

	sub_ = nh.subscribe("motor_vel", 10, MotorVelCallback); //topic que function
	sub2_ = nh.subscribe("trajectory", 10, TrajectoryCallback); //topic que function
	pub_ = nh.advertise<pharos_myrio::MotorVelStamped>("motor_cmd", 10); //topic que

	odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mobile_odom",1);


	ros::spin();

	return 0;
}




///////////////////////////Original Code///////////////////////////////////////////////////
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pharos_myrio/MotorVelStamped.h"
#include "pharos_motion/PointArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_;
ros::Subscriber sub2_;
ros::Publisher pub_;
ros::Publisher pub2_;
ros::Publisher odom_pub_;

pharos_motion::PointArray points_;
geometry_msgs::Point current_pos_;
nav_msgs::Odometry mobile_odom_;

bool isInit = false;
float desiredVel;

geometry_msgs::Point FindCurrentPosition(pharos_myrio::MotorVel vel, bool isInit)
{
	static geometry_msgs::Point p;

	if(!isInit){
		p.x = 0;
		p.y = 0;
		p.z = M_PI/2;
	}

	p.x += cos(p.z)*(vel.left+vel.right)/2*0.01;
	p.y += sin(p.z)*(vel.left+vel.right)/2*0.01;
	p.z += (vel.right-vel.left)/0.49*0.01;

	mobile_odom_.header.stamp = ros::Time::now();
	mobile_odom_.header.frame_id = "odom";
	mobile_odom_.child_frame_id = "mobile_odom";

	mobile_odom_.pose.pose.position.x = p.x;
	mobile_odom_.pose.pose.position.y = p.y;
	mobile_odom_.pose.pose.position.z = 0;

	// tf::Quaternion tf_Q;
	// geometry_msgs::Quaternion msg_Q;
	// tf_Q.setRPY(0, 0, p.z);
	// tf2::convert(msg_Q, tf_Q);
	mobile_odom_.pose.pose.orientation.x = 0;
	mobile_odom_.pose.pose.orientation.y = 0;
	mobile_odom_.pose.pose.orientation.z = 0;
	mobile_odom_.pose.pose.orientation.w = 1;

	odom_pub_.publish(mobile_odom_);

	return p;
}

geometry_msgs::Point FindDesiredPoint(geometry_msgs::Point p)
{
	static geometry_msgs::Point diff;
	float dist;
	float dist_min = 1;
	float dist_next = 1;
	int index;
	for(int i=0; i<points_.points.size(); i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist < dist_min){
			index = i;
			dist_min = dist;
		}
	}
	for(int i=index; i<points_.points.size(); i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist > 0.1){
			index = i;
			dist_next = dist;
		}
	}
	diff.x = points_.points[index].x - p.x;
	diff.y = points_.points[index].y - p.y;
	diff.z = atan2(diff.y, diff.x);
	return diff;
}

pharos_myrio::MotorVel FindMotorVel(geometry_msgs::Point curr, geometry_msgs::Point next)
{
	pharos_myrio::MotorVel vel;

	std::cout<<"next.z: "<<next.z<<std::endl;
	std::cout<<"curr.z: "<<curr.z<<std::endl;
	float theta_dot = (next.z-curr.z)/0.01;

	std::cout<<"theta_dot: "<<theta_dot<<std::endl;
	if(theta_dot > 0){
		vel.left = desiredVel - theta_dot * 0.49;
		vel.right = desiredVel;
	}else{
		vel.left = desiredVel;
		vel.right = desiredVel + theta_dot * 0.49;
	}

	return vel;
}

void MotorVelCallback(const pharos_myrio::MotorVelStamped::ConstPtr& msg)
{
	std::cout<<"MotorVelCallback start\n";
	static pharos_myrio::MotorVelStamped cmd;
	cmd.vel.left = 0;
	cmd.vel.right = 0;

	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "mobile_robot";

	cmd.state = 1;

	std::cout<<"FindCurrentPosition start\n";
	geometry_msgs::Point currentPoint = FindCurrentPosition(msg->vel, isInit);

	if(isInit){
		std::cout<<"FindDesiredPoint start\n";
		geometry_msgs::Point nextPoint = FindDesiredPoint(currentPoint);
		std::cout<<"FindMotorVel start\n";
		pharos_myrio::MotorVel motorVel = FindMotorVel(currentPoint, nextPoint);
		std::cout<<"FindMotorVel end\n";

		cmd.vel.left = motorVel.left;
		cmd.vel.right = motorVel.right;

		pub_.publish(cmd);
	}
	std::cout<<"MotorVelCallback end\n";
}

void TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg)
{
	isInit = true;

	static geometry_msgs::Point point;

	for(int i=0; i < msg->poses.back().header.seq; i++){
		point.x = msg->poses[i].pose.position.x;
		point.y = msg->poses[i].pose.position.y;
		points_.points.push_back(point);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_motion");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("vel", desiredVel, 0.1);

	sub_ = nh.subscribe("motor_vel", 10, MotorVelCallback); //topic que function
	sub2_ = nh.subscribe("trajectory", 10, TrajectoryCallback); //topic que function
	pub_ = nh.advertise<pharos_myrio::MotorVelStamped>("motor_cmd", 10); //topic que

	odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mobile_odom",1);


	ros::spin();

	return 0;
}



////////////////////////////////////////V3//////////////////////////////

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pharos_myrio/MotorVelStamped.h"
#include "pharos_motion/PointArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_;
ros::Subscriber sub2_;
ros::Publisher pub_;
ros::Publisher pub2_;
ros::Publisher odom_pub_;

pharos_motion::PointArray points_;
geometry_msgs::Point current_pos_;
nav_msgs::Odometry mobile_odom_;

bool isInit = false;
float desiredVel;

geometry_msgs::Point FindCurrentPosition(pharos_myrio::MotorVel vel, bool isInit)
{
	static geometry_msgs::Point p;

	if(!isInit){
		p.x = 0;
		p.y = 0;
		p.z = M_PI/2;
	}

	p.x += cos(p.z)*(vel.left+vel.right)/2*0.01;
	p.y += sin(p.z)*(vel.left+vel.right)/2*0.01;
	p.z += (vel.right-vel.left)/0.49*0.01;

	mobile_odom_.header.stamp = ros::Time::now();
	mobile_odom_.header.frame_id = "odom";
	mobile_odom_.child_frame_id = "mobile_odom";

	mobile_odom_.pose.pose.position.x = p.x;
	mobile_odom_.pose.pose.position.y = p.y;
	mobile_odom_.pose.pose.position.z = 0;

	mobile_odom_.pose.pose.orientation.x = 0;
	mobile_odom_.pose.pose.orientation.y = 0;
	mobile_odom_.pose.pose.orientation.z = 0;
	mobile_odom_.pose.pose.orientation.w = 1;

	odom_pub_.publish(mobile_odom_);

	return p;
}

geometry_msgs::Point FindDesiredPoint(geometry_msgs::Point p)
{
	static geometry_msgs::Point diff;
	float dist;
	float dist_min = 1;
	float dist_next = 1;
	int index;
	for(int i=0; i<points_.points.size(); i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist < dist_min){
			index = i;
			dist_min = dist;
		}
	}
	for(int i=index; i<points_.points.size(); i++){
		dist = sqrt(pow(points_.points[i].x-p.x,2)+pow(points_.points[i].y-p.y,2));
		if(dist > 0.1){
			index = i;
			dist_next = dist;
		}
	}
	diff.x = points_.points[index].x - p.x;
	diff.y = points_.points[index].y - p.y;
	diff.z = atan2(diff.x, diff.y);
	return diff;
}

pharos_myrio::MotorVel FindMotorVel(geometry_msgs::Point curr, geometry_msgs::Point next)
{
	pharos_myrio::MotorVel vel;

	std::cout<<"next.z: "<<next.z<<std::endl;
	std::cout<<"curr.z: "<<curr.z<<std::endl;
	float theta_dot = (next.z-curr.z)/0.01;

	std::cout<<"theta_dot: "<<theta_dot<<std::endl;
	if(theta_dot > 2*desiredVel/0.49){
		vel.right = desiredVel;
		vel.left = -desiredVel;
	}else if(theta_dot < - 2*desiredVel/0.49)
	{
		vel.right = -desiredVel;
		vel.left = desiredVel;
	}else
	{
		if(theta_dot > 0){
			vel.right = desiredVel ;
			vel.left = desiredVel- theta_dot * 0.49;
		}else{
			vel.right = desiredVel - theta_dot * 0.49;
			vel.left = desiredVel;
		}

	}
	return vel;
}

void MotorVelCallback(const pharos_myrio::MotorVelStamped::ConstPtr& msg)
{
	std::cout<<"MotorVelCallback start\n";
	static pharos_myrio::MotorVelStamped cmd;
	cmd.vel.left = 0;
	cmd.vel.right = 0;

	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "mobile_robot";

	cmd.state = 1;

	std::cout<<"FindCurrentPosition start\n";
	geometry_msgs::Point currentPoint = FindCurrentPosition(msg->vel, isInit);

	if(isInit){
		std::cout<<"FindDesiredPoint start\n";
		geometry_msgs::Point nextPoint = FindDesiredPoint(currentPoint);
		std::cout<<"FindMotorVel start\n";
		pharos_myrio::MotorVel motorVel = FindMotorVel(currentPoint, nextPoint);
		std::cout<<"FindMotorVel end\n";

		cmd.vel.left = motorVel.left;
		cmd.vel.right = motorVel.right;

		pub_.publish(cmd);
	}
	std::cout<<"MotorVelCallback end\n";
}

void TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg)
{
	isInit = true;

	static geometry_msgs::Point point;

	for(int i=0; i < msg->poses.back().header.seq; i++){
		point.x = msg->poses[i].pose.position.x;
		point.y = msg->poses[i].pose.position.y;
		points_.points.push_back(point);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_motion");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<float>("vel", desiredVel, 0.1);

	sub_ = nh.subscribe("motor_vel", 10, MotorVelCallback); //topic que function
	sub2_ = nh.subscribe("trajectory", 10, TrajectoryCallback); //topic que function
	pub_ = nh.advertise<pharos_myrio::MotorVelStamped>("motor_cmd", 10); //topic que

	odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mobile_odom",1);


	ros::spin();

	return 0;
}