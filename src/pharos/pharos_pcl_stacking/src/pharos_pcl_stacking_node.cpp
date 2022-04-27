#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"

// PointCloud2 to pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h> //to publish pointcloud made by pcl ...maybe


#include <pcl/filters/passthrough.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include <vector>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Subscriber clock_sub_;
ros::Subscriber sub_rover_odom;
ros::Subscriber sub_pcd;
ros::Subscriber sub_request_;
ros::Publisher pub_rover_odom;
ros::Publisher pub_PC;

sensor_msgs::PointCloud2 stacked_msg;

pcl::PCLPointCloud2 pcl_stack_;
pcl::PointCloud<pcl::PointXYZI>::Ptr CloudStack(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTemp(new pcl::PointCloud<pcl::PointXYZI>);


int cloudNum_;
float cloudSize_;
float posThreshold_;
float angThreshold_;

bool isGazebo_;
ros::Time GazeboClock_;

ros::Time Clock()
{
	if(isGazebo_) return GazeboClock_;
	return ros::Time::now();
}

void ClockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	GazeboClock_ = msg->clock;
}

void CropPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
	pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, cloudSize_);
    pass.filter(*input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-cloudSize_/2, cloudSize_/2);
    pass.filter(*input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1, 1);
    pass.filter(*input);
}

void VlpPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	static tf::TransformListener listener;
	static sensor_msgs::PointCloud2 output;

	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/haptic_field", "/velodyne", ros::Time(0), transform);
	}catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(0.01).sleep();
	}
	pcl_ros::transformPointCloud("/haptic_field", transform, *msg, output);
    pcl::fromROSMsg(output, *Cloud);
	CropPCD(Cloud);
}


void updatePCL()
{
	static std::vector<int> cloudSize;
	if(cloudSize.size() < cloudNum_){
		cloudSize.push_back(Cloud->size());
	}else{
		for(int i=cloudNum_-1; i>0; i--){
			cloudSize.at(i) = cloudSize.at(i-1);
		}
		cloudSize.at(0) = Cloud->size();
	}

	// int maxSize = 2500*10;
	int maxSize = 0;
	for(std::vector<int>::iterator it = cloudSize.begin(); it != cloudSize.end(); ++it) maxSize += *it;

	for(int i = CloudStack->size()+Cloud->size(); i > maxSize; i--){
       CloudStack->erase(CloudStack->end());
	}


	static tf::TransformListener listener2;
	static tf::StampedTransform odom2rover;
	static tf::Transform rover2odom;

	rover2odom = odom2rover;
	rover2odom = rover2odom.inverse();


	try{
		listener2.lookupTransform("/haptic_field", "/odom", ros::Time(0), odom2rover);
	}catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(0.01).sleep();
	}

	pcl_ros::transformPointCloud(*CloudStack, *CloudTemp, odom2rover*rover2odom);


	*CloudStack = *Cloud;
	*CloudStack += *CloudTemp;
	//

	pcl::toROSMsg(*CloudStack, stacked_msg);
	stacked_msg.header.frame_id = "/haptic_field";
	pub_PC.publish(stacked_msg);
}

void stackOdomPub(geometry_msgs::Pose &pose)
{
	nav_msgs::Odometry odom;

	odom.header.stamp = Clock();
	odom.header.frame_id = "odom";
	odom.child_frame_id = "husky_move_every_10cm";

	odom.pose.pose = pose;

	pub_rover_odom.publish(odom);
}

void RoverOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static geometry_msgs::Pose pose, pose_old;
	pose = msg->pose.pose;

	float angle = sqrt(pow(pose.orientation.x - pose_old.orientation.x,2)
		+pow(pose.orientation.y - pose_old.orientation.y,2)
		+pow(pose.orientation.z - pose_old.orientation.z,2)
		+pow(pose.orientation.w - pose_old.orientation.w,2));
	float dist = sqrt(pow(pose.position.x - pose_old.position.x,2)
		+pow(pose.position.y - pose_old.position.y,2));

	if(dist > posThreshold_ | angle > angThreshold_){

		stackOdomPub(pose);

		updatePCL();

		pose_old = pose;
	}
}

void StackRequestCallback(const std_msgs::Bool::ConstPtr& msg){
	ROS_WARN("stack publish request");
	pub_PC.publish(stacked_msg);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<bool>("gazebo", isGazebo_, true);
	pnh.param<int>("cloud_num", cloudNum_, 10);
	pnh.param<float>("map_size", cloudSize_, 6);
	pnh.param<float>("pos_threshold", posThreshold_, 0.2);
	pnh.param<float>("ang_threshold", angThreshold_, 0.05);

	clock_sub_ = nh.subscribe("/clock", 10, ClockCallback); //topic que function
	sub_rover_odom = nh.subscribe("/odom/slam_ekf", 10, RoverOdomCallback); //topic que function
	sub_pcd = nh.subscribe("/velodyne_points", 10, VlpPointCloudCallback); //topic que function
	sub_request_ = nh.subscribe("/stack/request", 10, StackRequestCallback); //topic que function

	pub_rover_odom = nh.advertise<nav_msgs::Odometry>("/odom/husky_move_every_10cm", 10); //topic que
	pub_PC = nh.advertise<sensor_msgs::PointCloud2>("/stack/velodyne_points", 10); //topic que


	ros::spin();

	return 0;
}
