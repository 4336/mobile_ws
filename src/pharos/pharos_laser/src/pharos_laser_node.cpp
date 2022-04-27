#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub;
ros::Publisher pub;

sensor_msgs::LaserScan scan_;

#define VPoint velodyne_pointcloud::PointXYZIR
#define Point2 pcl::PointXYZIR

int num_neigbor_points_;
float std_multiplier_;
float cellSize_;

void Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	scan_ = *scan;
	memset(&scan_.intensities,0,sizeof(scan_.ranges));


	//sensor_msgs::LaserScan → sensor_msgs::PointCloud2
	static laser_geometry::LaserProjection projector;
	sensor_msgs::PointCloud2 pc2_dst;
	projector.projectLaser(scan_, pc2_dst,-1);
	pc2_dst.header.frame_id = "map";


	//sensor_msgs::PointCloud2 → pcl::PointCloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(pc2_dst,*cloud);


	//pcl::PointCloud -> 2D filtered pcl::PointCloud to remove outliers.
	//인접한 10중 표준편차가 1이상인 outlier를 제거한다. 


	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

	*ptr_cloud_filtered = *cloud;    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> filter;
    filter.setInputCloud (ptr_cloud_filtered);
    filter.setMeanK (num_neigbor_points_);
    filter.setStddevMulThresh (std_multiplier_);
    filter.filter(*ptr_cloud_filtered);
    *cloud_filtered = *ptr_cloud_filtered;
	std::cout<<num_neigbor_points_<<std::endl;

	//ROS로 다시 바꿔주기위해서 필요한 작업
	//sensor_msgs::PointCloud2 → pcl::PointCloud and voxel grid
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCLPointCloud2 cloud_v;
	pcl::VoxelGrid<pcl::PointXYZI> cluster;

	cluster.setInputCloud (cloud_filtered);
	cluster.setLeafSize (cellSize_, cellSize_, cellSize_);
	cluster.filter (*cloud_clustered); 
	pcl::toPCLPointCloud2(*cloud_clustered, cloud_v);

	sensor_msgs::PointCloud2 output_v;
	pcl_conversions::fromPCL(cloud_v, output_v);
	output_v.header.frame_id = "laser";

	static tf::TransformListener listener;
    static tf::StampedTransform transform;
    try{
		// listener.waitForTransform("/rover", "/laser", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/rover", "/laser", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.01).sleep();
    }
	sensor_msgs::PointCloud2 output_final;
	pcl_ros::transformPointCloud("/rover", transform, output_v, output_final);

	pub.publish(output_final);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_laser_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<int>("num_gain", num_neigbor_points_, 10);
	pnh.param<float>("std_gain", std_multiplier_, 1);
	pnh.param<float>("cell_size", cellSize_, 0.1);

	sub = nh.subscribe("scan", 10, Callback); //topic que function
	pub = nh.advertise<sensor_msgs::PointCloud2>("filteredPointCloud", 10); //topic que

	ros::spin();

	return 0;
}