#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

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
ros::Publisher pub_block_BKmap;
ros::Publisher pub_BKmap;
ros::Publisher pub_crop_test;

//Variables
pcl::PointCloud<pcl::PointXYZ> plane_cloud;
pcl_msgs::ModelCoefficients ros_coefficients;
pcl::PointCloud<pcl::PointXYZ> pcl_croped_height;
pcl::PointCloud<pcl::PointXYZ> block_bk_map;
pcl::PointCloud<pcl::PointXYZ> bk_map;
float block_num;
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
    // pass.setFilterLimits(-0.5,0.5);
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

void calculate_height (pcl::PointCloud<pcl::PointXYZ> pcl_croped, pcl_msgs::ModelCoefficients ros_coefficients, int pointNum)
{
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
        pcl_croped_height.points[i].x = x;
        pcl_croped_height.points[i].y = y;
        pcl_croped_height.points[i].z = z - (- 1/C * (A*x + B*y + D));
    } 
}

float Get_Variance (int i, int j, pcl::PointCloud<pcl::PointXYZ> pcl_croped_height, float block_size, int pointNum)
{
    pcl::PointCloud<pcl::PointXYZ> in_block_value;

    for (int idx = 0 ; idx < pointNum ; idx++)
    {
        if (0 + block_size * i <= pcl_croped_height.points[idx].x && pcl_croped_height.points[idx].x <= 0 + block_size * (i + 1 ))
        {
            if (2.5 - block_size * (j + 1) <= pcl_croped_height.points[idx].y && pcl_croped_height.points[idx].y <= 2.5 - block_size * j)
            {
                //make new pcl::PointXYZ
                in_block_value.push_back(pcl::PointXYZ(pcl_croped_height.points[idx].x, pcl_croped_height.points[idx].y, pcl_croped_height.points[idx].z));

                if(i==6&&j==0){
                    sensor_msgs::PointCloud2 crop_pcd;
                    pcl::toROSMsg(in_block_value, crop_pcd);
                    crop_pcd.header.stamp = ros::Time::now();
                    crop_pcd.header.frame_id = "haptic_field";
                    pub_crop_test.publish(crop_pcd);
                }
            }
        }
    }


    //Value for variance
    int n = in_block_value.points.size();
    float sum_z = 0;
    for (int idx2 = 0 ; idx2 < n ; idx2++)
    {
        sum_z += pcl_croped_height.points[idx2].z;
    }
    float mean = sum_z / n;

    //Calculate Variance
    float sum_diff = 0;
    for (int idx3 = 0 ; idx3 < n ; idx3++)
    {
        sum_diff += pow(pcl_croped_height.points[idx3].z - mean,2);
    }
    if(i==0&&j==6){
        std::cout<<"sum_z: "<<sum_z<<std::endl;
        std::cout<<"n: "<<n<<std::endl;
        std::cout<<"mean: "<<mean<<std::endl;
        std::cout<<"z: "<<pcl_croped_height.points[n-1].z<<std::endl;
    }
    float variance = sum_diff / (n - 1);
    float gain = pow(10,1);
    variance = gain * variance;
    //std::cout << i << j << " : " << variance << std::endl;

    return variance;
}

float get_BK (float x, float y, float radius, pcl::PointCloud<pcl::PointXYZ> block_bk_map, float block_num)
{
    float bk = 0;
    int cnt = 0;

    for (int i = 0 ; i < block_num ; i++)
    {
        double distance;
        distance = sqrt(pow(x - block_bk_map.points[i].x,2) + pow(y - block_bk_map.points[i].y,2));

        if(distance <= radius)
        {
            bk += block_bk_map.points[i].z;
            cnt++;
        }
    }
    bk /= cnt;
    return bk;
}

void make_map (const pcl::PointCloud<pcl::PointXYZ> pcl_croped_height, int pointNum)
{
    //BK block map
    // float block_num;
    float block_size = 5 / sqrt(block_num);

    block_bk_map.clear();
    
    for(int i = 0 ; i < sqrt(block_num) ; i++)
    {
        for(int j = 0 ; j < sqrt(block_num) ; j++)
        {
            float start_x = 2.5 / sqrt(block_num);
            float start_y = 2.5 - start_x;
            float block_x = start_x + i * block_size; 
            float block_y = start_y - j * block_size; 
            float block_bk = Get_Variance(i, j, pcl_croped_height, block_size, pointNum);
            block_bk_map.push_back(pcl::PointXYZ(block_x, block_y, block_bk));
            if( i==0 && j==6 )
            {
                std::cout <<"Variance : "<<Get_Variance(i, j, pcl_croped_height, block_size, pointNum) << std::endl;
            }
        }
    }



    //BK point Map
    float radius = block_size;
    // float i_number;
    // float j_number;

    bk_map.clear();

    for(int i = 0 ; i <= i_number ; i++)
    {
        for(int j = 0 ; j <= j_number ; j++)
        {
            float x = 0 + i * 5 / i_number; 
            float y = 2.5 - j * 5 / j_number; 
            float bk = get_BK (x, y, radius, block_bk_map, block_num);
            bk_map.push_back(pcl::PointXYZ(x, y, bk));
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


    //Calculate Height
    pcl_croped_height = *pcl_croped_ptr;
    calculate_height (pcl_croped, ros_coefficients, pointNum);


    //Make BK Map
    make_map (pcl_croped_height, pointNum);


    //pcl::PointCloud → sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 block_output;
    pcl::toROSMsg(block_bk_map, block_output);
    block_output.header.frame_id = "/haptic_field";
    pub_block_BKmap.publish(block_output);

    //pcl::PointCloud → sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(bk_map, output);
    output.header.frame_id = "/haptic_field";
    pub_BKmap.publish(output);

    //pcl::PointCloud → sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 plane;
    pcl::toROSMsg(plane_cloud, plane);
    plane.header.frame_id = "/haptic_field";
    pub_plane.publish(plane);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_BKmap");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

    pnh.param<float>("x_point", i_number, 30);
    pnh.param<float>("y_point", j_number, 30);
    pnh.param<float>("block_num", block_num, 49);

	sub_velodyne = nh.subscribe("/stack/velodyne_points", 10, Callback); //topic que function
	pub_coefficient = nh.advertise<pcl_msgs::ModelCoefficients>("coefficient", 10); //topic que
    pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane", 10); //topic que
    pub_block_BKmap = nh.advertise<sensor_msgs::PointCloud2>("Block_BK_map", 10); //topic que
    pub_BKmap = nh.advertise<sensor_msgs::PointCloud2>("BK_map", 10); //topic que
    pub_crop_test = nh.advertise<sensor_msgs::PointCloud2>("crop_test", 10); //topic que

	ros::spin();

	return 0;
}