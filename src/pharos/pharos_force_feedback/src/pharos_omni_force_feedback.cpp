#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include<algorithm>
#include<vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>

#include <omni_msgs/OmniFeedback.h>
#include <pharos_omni_msgs/OmniState.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h> 

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_Zmap;
ros::Subscriber sub_BKmap;
ros::Subscriber sub_end;
ros::Subscriber sub_handle;
ros::Publisher pub_forceFB;
ros::Publisher pub_haptic;
ros::Publisher pub_grid_;
ros::Publisher pub_stack_request_;
ros::Publisher pub_obstacle_map_;

pcl::PointCloud<pcl::PointXYZ> GridTest_;

pcl::PointCloud<pcl::PointXYZI> Zmap_;
pcl::PointCloud<pcl::PointXYZI> BKmap_;

omni_msgs::OmniFeedback omniFB_;

geometry_msgs::Vector3 pos_;
geometry_msgs::Vector3 pos_old_;

geometry_msgs::Vector3Stamped force_output;
geometry_msgs::Vector3Stamped old_force;

visualization_msgs::Marker forceMarker_;

pharos_omni_msgs::OmniState OmniState_;

sensor_msgs::PointCloud2 gridTestMsg_;

sensor_msgs::PointCloud2 ObstacleMsg_;

ros::Time init_time_;

bool is_Z_init_ = false;
bool is_BK_init_ = false;

int pointNum;

long cnt_ = 0;

void ForceMarkerInit(visualization_msgs::Marker *force)
{
	force->header.frame_id = "start_point";
	force->header.stamp = ros::Time();
	force->ns = "haptic_feedback";
	force->id = 0;
	force->type = visualization_msgs::Marker::ARROW;
	force->action = visualization_msgs::Marker::ADD;
	force->pose.position.x = 0;
	force->pose.position.y = 0;
	force->pose.position.z = 0;
	force->pose.orientation.x = 0.0;
	force->pose.orientation.y = 0.0;
	force->pose.orientation.z = 0.0;
	force->pose.orientation.w = 1.0;
	force->scale.x = 0.1;
	force->scale.y = 0.1;
	force->scale.z = 0.1;
	force->color.a = 1.0; // Don't forget to set the alpha!
	force->color.r = 1.0;
	force->color.g = 0.0;
	force->color.b = 0.0;

	old_force.vector.x = 0;
	old_force.vector.y = 0;
	old_force.vector.z = 0;
	//only if using a MESH_RESOURCE marker type:
	// marker_mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

void Callback_Zmap(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	is_Z_init_ = true;
	//sensor_msgs::PointCloud2 → pcl::PointCloud
	pcl::fromROSMsg(*msg, Zmap_);
}

void Callback_BKmap(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	is_BK_init_ = true;
	//sensor_msgs::PointCloud2 → pcl::PointCloud
	pcl::fromROSMsg(*msg, BKmap_);

	//normalize
	float max=0;
	for(int i = 0; i < BKmap_.points.size(); i++)
	{
		if(max < BKmap_.points[i].z) max = BKmap_.points[i].z;
	}
	for(int i = 0; i < BKmap_.points.size(); i++)
	{
		BKmap_.points[i].z = BKmap_.points[i].z / max;
	}
}

void OmniStateCallback(const pharos_omni_msgs::OmniState::ConstPtr& msg)
{
	OmniState_ = *msg;
}

float interpolation (float x, float y, pcl::PointCloud<pcl::PointXYZI> map)
{

	//Floor the value 
	//resolution is 0.1 so, 소수 둘째자리 까지 
	float	x_prime = floor(x * 10) / 10;
	float y_prime = floor(y * 10) / 10;

	//Find map index
	int idx = ((2.5 - y_prime) / 5) * 50 + (((x_prime) / 5) * 50) * 51;

	//Get near point x,y position of map
	float x1 = map.points[idx].x;
	float y1 = map.points[idx].y;
	float x2 = map.points[idx + 52].x;
	float y2 = map.points[idx + 52].y;

	//Do Bilinear Interpolation
	float w11 = (x2 - x)*(y2 - y) / (x2 - x1)*(y2 - y1)*100;
	float w12 = (x2 - x)*(y - y1) / (x2 - x1)*(y2 - y1)*100;
	float w21 = (x - x1)*(y2 - y) / (x2 - x1)*(y2 - y1)*100;
	float w22 = (x - x1)*(y - y1) / (x2 - x1)*(y2 - y1)*100;

	//Calculate the return value(Z or BK)
	float output = w11 * map.points[idx + 51 + 1].z 
								+ w12 * map.points[idx + 1].z 
								+ w21 * map.points[idx + 51].z 
								+ w22 * map.points[idx].z;



	GridTest_.clear();
	GridTest_.push_back(pcl::PointXYZ(map.points[idx].x, map.points[idx].y, map.points[idx].z));
	GridTest_.push_back(pcl::PointXYZ(map.points[idx+1].x, map.points[idx+1].y, map.points[idx+1].z));
	GridTest_.push_back(pcl::PointXYZ(map.points[idx+51].x, map.points[idx+51].y, map.points[idx+51].z));
	GridTest_.push_back(pcl::PointXYZ(map.points[idx+52].x, map.points[idx+52].y, map.points[idx+52].z));

	pcl::toROSMsg(GridTest_,gridTestMsg_);

	gridTestMsg_.header.stamp = ros::Time::now();
	gridTestMsg_.header.frame_id = "haptic_field";

	return output;
}

float tactle_feedback(float z_value, float bk_value)
{
	float z_scaled = z_value * 0.5;

	//About z_value(slop, bump, etc)
	float z_force = 0;
	float z_kp = 30;
	float z_wall_offset = 0.05;

	if(z_scaled + z_wall_offset > pos_.z){
		z_force = (z_scaled + z_wall_offset - pos_.z)*z_kp;
	}else z_force = 0;

	//About bk_value(sinusoidal)
	float gain_bk = 1;
	float amplitude = gain_bk * bk_value;
	ros::Duration duration = ros::Time::now() - init_time_;
	float time = duration.toSec() * 2 * 3.1415926 * 10;
	float wave = amplitude * sin(time);

	if (fabs(wave) < 1) ;
	else wave = 0;

	float output_z = z_force + wave;

	return output_z;
}

void force_feedback (pcl::PointCloud<pcl::PointXYZI> map, float x, float y, geometry_msgs::Vector3Stamped old_force)
{
	geometry_msgs::Vector3Stamped force;
	geometry_msgs::Vector3 pointforce;
	pcl::PointCloud<pcl::PointXYZ> xy_force_point;

	//Find the higher then 1std
	int n = map.points.size();
    float sum_z = 0;
    for (int i = 0 ; i < n ; i++)
    {
        sum_z += map.points[i].z;
    }
    float mean = sum_z / n;

    //Calculate std
    float diff_sum = 0;
    for (int i = 0 ; i < n ; i++)
    {
        diff_sum += pow(map.points[i].z - mean,2);
    }
    float std = sqrt(diff_sum / (n - 1));


	//Find the wall
	for(int i = 0; i < n; i++)
	{
		if(map.points[i].z > std * 2)
		{
			xy_force_point.push_back(pcl::PointXYZ(map.points[i].x, map.points[i].y, map.points[i].z));
		} 
	}

	//Add the endline
	//left line
	for(int idx = 51; idx < 2601; idx = idx + 51)
	{
		xy_force_point.push_back(pcl::PointXYZ(map.points[idx].x, map.points[idx].y, map.points[idx].z));
	}
	//right line
	for(int idx = 101; idx < 2601; idx = idx + 51)
	{
		xy_force_point.push_back(pcl::PointXYZ(map.points[idx].x, map.points[idx].y, map.points[idx].z));
	}
	//upper line
	for(int idx = 2450; idx < 2601; idx++)
	{
		xy_force_point.push_back(pcl::PointXYZ(map.points[idx].x, map.points[idx].y, map.points[idx].z));
	}

	pcl::toROSMsg(xy_force_point,ObstacleMsg_);
	ObstacleMsg_.header.frame_id = "haptic_field";
	ObstacleMsg_.header.stamp = ros::Time::now();
	pub_obstacle_map_.publish (ObstacleMsg_);

	//Calculate Force
	force.vector.x = 0;
	force.vector.y = 0;
	int pointNum = xy_force_point.points.size();


#if 0
	for(int i = 0; i < pointNum; i++)
	{
		int cnt = 0;
		double maxDist = 0.2;
		double distance;
		distance = sqrt(pow(x - xy_force_point.points[i].x, 2) + pow(y - xy_force_point.points[i].y, 2));
		if(distance <= maxDist)
		{
			cnt++;
			double gain;
			gain = maxDist/distance*0.1;
			pointforce.x = gain * (x - xy_force_point.points[i].x)/fabs(x - xy_force_point.points[i].x);
			pointforce.y = gain * (y - xy_force_point.points[i].y)/fabs(y - xy_force_point.points[i].y);

			force.vector.x += pointforce.x;
			force.vector.y += pointforce.y;
		}
		if(cnt > 1){
			force.vector.x /= cnt;
			force.vector.y /= cnt;
		}
	}

	//Make the force limits
	float maxForce = 1;
	float norm = sqrt(pow(force.vector.x,2) + pow(force.vector.y,2));
	if(norm > maxForce){
			force.vector.x *= maxForce/norm;
			force.vector.y *= maxForce/norm;
	}
	//Pass the LPF
	float a = 0.1;
	a = 1;
	force_output.vector.x = a * force.vector.x + (1-a) * old_force.vector.x;
	force_output.vector.y = a * force.vector.y + (1-a) * old_force.vector.y;

#else

	float radius = 0.2;
	float maxForce = 2;

	int RenderPointcnt = 0;
	std::vector<std::pair<float, int>> dist_index;

	for(int i = 0; i < pointNum; i++)
	{
		float dist = sqrt(pow(x - xy_force_point.points[i].x, 2) + pow(y - xy_force_point.points[i].y, 2));
		if(dist < radius)
		{
			dist_index.push_back(std::pair<float, int>(dist, i));

			RenderPointcnt++;
		}
	}
	std::sort(dist_index.begin(), dist_index.end());


	float xF = 0;
	float yF = 0;

	float gain = 1;
	for(int i=0; i<RenderPointcnt; i++){
		if(RenderPointcnt > 1 || i == RenderPointcnt-1) gain /= 2;
		int index = dist_index[i].second; //index

		float xDiff = x - xy_force_point.points[index].x;
		float yDiff = y - xy_force_point.points[index].y;
		xF += (radius - dist_index[i].first) * cos(atan2(yDiff, xDiff)) / radius * maxForce;
		yF += (radius - dist_index[i].first) * sin(atan2(yDiff, xDiff)) / radius * maxForce;
	}
	//Pass the LPF
	float a = 0.05;
	a = 1;
	force_output.vector.x = a * xF + (1-a) * old_force.vector.x;
	force_output.vector.y = a * yF + (1-a) * old_force.vector.y;


#endif

	std::cout << "=========" << std::endl;
	std::cout << "x force : " << force_output.vector.x << std::endl;
	std::cout << "yy force : " << force_output.vector.y << std::endl;

}

void EndPointCallback(const geometry_msgs::Vector3Stamped::ConstPtr& handle)
{
	pos_old_ = pos_;
	//Get position
	pos_.x = handle->vector.x;
	pos_.y = handle->vector.y;
	pos_.z = handle->vector.z;
	
	cnt_++;

	//Get Z value at that point
	float z_value;
	if(is_Z_init_)
	{
		z_value = interpolation(pos_.x, pos_.y, Zmap_);
	}else
	{
		ROS_WARN("Z_MAP");
		z_value = 0;
		return;
	}


	//Get BK value at that point
	float bk_value;
	if(is_BK_init_)
	{
		bk_value = interpolation(pos_.x, pos_.y, BKmap_);
	}else
	{
		ROS_WARN("BK_MAP");
		bk_value = 0;
		return;
	}

	pub_grid_.publish(gridTestMsg_);


	//Give X Y feedback (virtual wall)
	float force_x, force_y;
	force_feedback(Zmap_, pos_.x, pos_.y, old_force);
	old_force = force_output;


	//Give Tactle feedback (Z slope + vibration)
	// force_output.vector.z = tactle_feedback(z_value, bk_value);


	//Publish force feedback
	if(OmniState_.index){
		omniFB_.force.x = -force_output.vector.y;
		omniFB_.force.y = force_output.vector.z;
		omniFB_.force.z = -force_output.vector.x;
	}else{
		omniFB_.force.x = 0;
		omniFB_.force.y = 0;
		omniFB_.force.z = 0;
	}
	pub_forceFB.publish(omniFB_);


	//Make and Publish Force Marker (Visualization)
	forceMarker_.scale.x = sqrt(pow(force_output.vector.x,2) + pow(force_output.vector.y,2));

	float theta = atan2(force_output.vector.y, force_output.vector.x);

	Eigen::Quaternionf q;
	q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());

	forceMarker_.pose.orientation.x = q.x();
	forceMarker_.pose.orientation.y = q.y();
	forceMarker_.pose.orientation.z = q.z();
	forceMarker_.pose.orientation.w = q.w();

	forceMarker_.header.stamp = ros::Time::now();
	forceMarker_.header.frame_id = "haptic_field";
	forceMarker_.pose.position.x = handle->vector.x;
	forceMarker_.pose.position.y = handle->vector.y;
	forceMarker_.pose.position.z = 0;
	pub_haptic.publish(forceMarker_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pharos_force_feedback");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");


	sub_Zmap = nh.subscribe<sensor_msgs::PointCloud2>("/Z_map", 10, Callback_Zmap); 
	sub_BKmap = nh.subscribe<sensor_msgs::PointCloud2>("/BK_map", 10, Callback_BKmap); 
	sub_end = nh.subscribe<geometry_msgs::Vector3Stamped>("/omni/end_point", 10, EndPointCallback);	// intergrated haptic point(last path point)
	sub_handle = nh.subscribe<pharos_omni_msgs::OmniState>("/omni/state", 10, OmniStateCallback);	// state, 
	pub_forceFB = nh.advertise<omni_msgs::OmniFeedback>("/omni/force_feedback", 10); //topic que
	pub_haptic = nh.advertise<visualization_msgs::Marker>("/odom/haptic_feedback",1);
	pub_grid_ = nh.advertise<sensor_msgs::PointCloud2>("/grid_search",1);
	pub_stack_request_ = nh.advertise<std_msgs::Bool>("/stack/request", 10); //topic que

	pub_obstacle_map_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_map", 10); //topic que

	std_msgs::Bool request;
	request.data = true;
	pub_stack_request_.publish(request);
	ROS_WARN("stack request");

	ForceMarkerInit(&forceMarker_);

	init_time_ = ros::Time::now();

	ros::spin();

	return 0;
}