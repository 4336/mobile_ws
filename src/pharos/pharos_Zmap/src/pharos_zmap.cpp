#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Bool.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_ros/transforms.h>

#include <pcl/filters/passthrough.h>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//Topic Nodes
ros::Subscriber sub_velodyne;
ros::Publisher pub_coefficient;
ros::Publisher pub_plane;
ros::Publisher pub_Zmap;
ros::Publisher pub_stack_request_;

//Variables
pcl::PointCloud<pcl::PointXYZ> plane_cloud;
pcl_msgs::ModelCoefficients ros_coefficients;
pcl::PointCloud<pcl::PointXYZ> z_map;
float radius;
float i_number;
float j_number;

//Functions
void CropPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0,5.0);
    pass.filter(*input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.5,2.5);
    pass.filter(*input);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-1.0,1.0);
    // pass.filter(*input);
}

void plane_detect (const pcl::PointCloud<pcl::PointXYZ> pcl_croped,  int pointNum)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (pcl_croped.makeShared ());
    seg.segment (inliers, coefficients);

    // Publish the model coefficients
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub_coefficient.publish (ros_coefficients);

    //Plane Formula
    //0 = Ax + By + Cz + D
    //z = - 1/C * (Ax + By + D)
    float A, B, C, D;
    A = ros_coefficients.values[0];
    B = ros_coefficients.values[1];
    C = ros_coefficients.values[2];
    D = ros_coefficients.values[3];

    for(int i = 0 ; i < pointNum; i++)
    {
        float x, y, z;
        x = pcl_croped.points[i].x;
        y = pcl_croped.points[i].y;
        z = pcl_croped.points[i].z;
        plane_cloud.points[i].x = x;
        plane_cloud.points[i].y = y;
        plane_cloud.points[i].z = (- 1/C * (A*x + B*y + D));
        //std::cout << plane_cloud.points[i].z << std::endl;
    }
}

float get_Z (float x, float y, float radius, pcl::PointCloud<pcl::PointXYZ> pcl_croped, pcl_msgs::ModelCoefficients plane_coefficients, int pointNum)
{
    float z = 0;
    int cnt = 0;

    for (int i = 0 ; i < pointNum ; i++)
    {
        double distance;
        distance = sqrt(pow(x - pcl_croped.points[i].x,2) + pow(y - pcl_croped.points[i].y,2));

        if(distance <= radius)
        {
            z += pcl_croped.points[i].z;
            cnt++;
        }
    }
    if(cnt == 0)
    {
        float A, B, C, D;
        A = plane_coefficients.values[0];
        B = plane_coefficients.values[1];
        C = plane_coefficients.values[2];
        D = plane_coefficients.values[3];

        z = (- 1/C * (A*x + B*y + D));
        cnt = 1;
    }

    z /= cnt;
    return z;
}

void make_map (const pcl::PointCloud<pcl::PointXYZ> pcl_croped, pcl_msgs::ModelCoefficients plane_coefficients, int pointNum)
{
    // float radius;
    // float i_number;
    // float j_number;

    z_map.clear();

    for(int i = 0 ; i <= i_number ; i++)
    {
        for(int j = 0 ; j <= j_number ; j++)
        {
            float x = 0 + i * 5 / i_number ; 
            float y = 2.5 - j * 5 / j_number; 
            float z = get_Z (x, y, radius, pcl_croped, plane_coefficients, pointNum);
            z_map.push_back(pcl::PointXYZ(x, y, z));
        }
    }
}

void Callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	//Transform the frame
	static tf::TransformListener listener;
    static tf::StampedTransform transform;
    try{
        // listener.waitForTransform("/rover", "/laser", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/haptic_field", "/velodyne", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.01).sleep();
    }
    sensor_msgs::PointCloud2 transformed_input;
    pcl_ros::transformPointCloud("/haptic_field", transform, *input, transformed_input);


	//sensor_msgs::PointCloud2 → pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(transformed_input, pcl_cloud);


    //Crop the region (Make ROI)
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_croped_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *pcl_croped_ptr = pcl_cloud;
    CropPCD(pcl_croped_ptr);
    pcl::PointCloud<pcl::PointXYZ> pcl_croped;
    pcl_croped = *pcl_croped_ptr;


    //Calculate number of point
    int pointNum;
    pointNum = pcl_croped.points.size();


    //Detect the plane using RANSAC
    plane_cloud = *pcl_croped_ptr;
    plane_detect (pcl_croped, pointNum);


    //Make Z Map
    make_map (pcl_croped, ros_coefficients, pointNum);


    //pcl::PointCloud → sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(z_map, output);
    output.header.frame_id = "/haptic_field";
    pub_Zmap.publish(output);

    //pcl::PointCloud → sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 plane;
    pcl::toROSMsg(plane_cloud, plane);
    plane.header.frame_id = "/haptic_field";
    pub_plane.publish(plane);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_Zmap");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

    pnh.param<float>("x_point", i_number, 50);
    pnh.param<float>("y_point", j_number, 50);
    pnh.param<float>("radius", radius, 0.1);

	sub_velodyne = nh.subscribe("/stack/velodyne_points", 1, Callback); //topic que function
	pub_coefficient = nh.advertise<pcl_msgs::ModelCoefficients>("coefficient", 10); //topic que
    pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane", 10); //topic que
    pub_Zmap = nh.advertise<sensor_msgs::PointCloud2>("Z_map", 10); //topic que
    pub_stack_request_ = nh.advertise<std_msgs::Bool>("/stack/request", 10); //topic que

    // std_msgs::Bool request;
    // request.data = true;
    // pub_stack_request_.publish(request);

	ros::spin();

	return 0;
}