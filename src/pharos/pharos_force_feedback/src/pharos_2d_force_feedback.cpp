#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber sub_lidar;
ros::Subscriber sub_end;
ros::Subscriber sub_handle;
ros::Publisher pub;
ros::Publisher pub_haptic;

sensor_msgs::PointCloud lidar;
Eigen::MatrixXf pointdata(2,2000);

geometry_msgs::Vector3Stamped force_output;
geometry_msgs::Vector3Stamped haptic_output;
geometry_msgs::Vector3Stamped force;
geometry_msgs::Vector3Stamped old_force;
geometry_msgs::Vector3 pointforce;

visualization_msgs::Marker forceMarker_;

int pointNum;



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

void Callback_Lidar(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, lidar);
	Eigen::MatrixXf pointclouddata(2, lidar.points.size());
	pointdata = pointclouddata;
	for(int i = 0 ; i < lidar.points.size(); i++)
	{
	    pointdata(0,i) = lidar.points[i].x;
	    pointdata(1,i) = lidar.points[i].y;
	}
	pointNum = lidar.points.size();

}

void HapticCallback(const geometry_msgs::Vector3Stamped::ConstPtr& handle)
{
	haptic_output.vector.z = handle->vector.z;
	pub.publish(haptic_output);
}

void EndPointCallback(const geometry_msgs::Vector3Stamped::ConstPtr& handle)
{
	//write a code that calculate feedback
	//Using IF function (beteween handle_pos and each point's x,y position -> then, calculate force
	//열 부분에 i 사용하여 반복문 이용하고 특정 기준점 안에 들어오면 둘 사이의 거리에 반비례한 상수값을 피드백 힘으로 선언 
	//한번 반복문이 끝나면 현재 핸들 위치로부터 특정 기준점 안에 들어오는 모든 힘 선언함 
	//반복문 안에서 벡터의 합이 된 총 힘을 게산 
	//총 힘을 퍼블리시

	// std::cout << "1" << std::endl;

	Eigen::VectorXf handledata(2);
    handledata(0) = handle->vector.x;
    handledata(1) = handle->vector.y;

    force.header.stamp = ros::Time::now();
    force.header.frame_id = "haptic_device";

	force.vector.x = 0;
	force.vector.y = 0;
	for(int i = 0; i < pointNum; i++)
	{
		int cnt = 0;
		double maxDist = 0.5;
		double distance;
		distance = sqrt(pow(handledata(0) - pointdata(0,i),2) + pow(handledata(1) - pointdata(1,i),2));
		if(distance <= maxDist)
		{
			cnt++;
			double gain;
			gain = maxDist/distance*0.1;
			pointforce.x = gain * (handledata(0) - pointdata(0,i))/fabs(handledata(0) - pointdata(0,i));
			pointforce.y = gain * (handledata(1) - pointdata(1,i))/fabs(handledata(1) - pointdata(1,i));

			force.vector.x += pointforce.x;
			force.vector.y += pointforce.y;
		}
		if(cnt > 1){
			force.vector.x /= cnt;
			force.vector.y /= cnt;
		}
	}
	float maxForce = 1;
	float norm = sqrt(pow(force.vector.x,2) + pow(force.vector.y,2));
	if(norm > maxForce){
			force.vector.x *= maxForce/norm;
			force.vector.y *= maxForce/norm;
	}

	//for(int a = 0; a<10; ++a){
		float a = 0.1;
		a = 1;
		force_output.vector.x = a * force.vector.x + (1-a) * old_force.vector.x;
		force_output.vector.y = a * force.vector.y + (1-a) * old_force.vector.y;
		// pub.publish(force_output);
		// std::cout << force.vector.x <<" "<< force.vector.y <<" "<< force.vector.z << std::endl;
	//}
	old_force = force_output;

	haptic_output.vector.x = -force_output.vector.y;
	haptic_output.vector.y = force_output.vector.x;


	//Make Force Marker (Visualization)
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
	forceMarker_.header.frame_id = "rover";
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

	std::string end_point;
	std::string force_feedback;

	nh.param("end_point", end_point, std::string("haptic/end_point"));
	nh.param("force_feedback", force_feedback, std::string("force_feedback"));

	sub_lidar = nh.subscribe<sensor_msgs::PointCloud2>("filteredPointCloud", 10, Callback_Lidar); 
	sub_end = nh.subscribe<geometry_msgs::Vector3Stamped>(end_point, 10, EndPointCallback);
	sub_handle = nh.subscribe<geometry_msgs::Vector3Stamped>("haptic_device", 10, HapticCallback);
	pub = nh.advertise<geometry_msgs::Vector3Stamped>(force_feedback, 10); //topic que
	pub_haptic = nh.advertise<visualization_msgs::Marker>("/odom/haptic_feedback",1);

	ForceMarkerInit(&forceMarker_);

	ros::spin();

	return 0;
}