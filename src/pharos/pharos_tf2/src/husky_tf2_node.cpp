#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/JointState.h>


geometry_msgs::TransformStamped tf_stamped_;

typedef struct tf_param_{
    std::string frame_id;
    std::string child_frame_id;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    geometry_msgs::Transform transform;
}tf_param_;

void Map_TF_Init(tf2_ros::StaticTransformBroadcaster &static_br, const nav_msgs::OdometryConstPtr &Odom){
    static bool isInit = false;
    if(!isInit){
        isInit = true;
        static geometry_msgs::Transform transform;

        transform.translation.x = Odom->pose.pose.position.x;
        transform.translation.y = Odom->pose.pose.position.y;
        transform.translation.z = Odom->pose.pose.position.z;
        transform.rotation = Odom->pose.pose.orientation;

        tf_stamped_.header.stamp = ros::Time::now();
        tf_stamped_.header.frame_id = "map";
        tf_stamped_.child_frame_id = "odom";
        tf_stamped_.transform = transform;

        static_br.sendTransform(tf_stamped_);
        // std::cout<<Odom->child_frame_id<<std::endl;
    }
}

void Odometry2TF(tf2_ros::TransformBroadcaster &br, const nav_msgs::OdometryConstPtr &Odom){
    static geometry_msgs::Transform transform;

    transform.translation.x = Odom->pose.pose.position.x;
    transform.translation.y = Odom->pose.pose.position.y;
    transform.translation.z = Odom->pose.pose.position.z;
    transform.rotation = Odom->pose.pose.orientation;

    tf_stamped_.header.stamp = ros::Time::now();
    tf_stamped_.header.frame_id = Odom->header.frame_id;
    tf_stamped_.child_frame_id = Odom->child_frame_id;
    tf_stamped_.transform = transform;

    br.sendTransform(tf_stamped_);
    // std::cout<<Odom->child_frame_id<<std::endl;
}

void UpdateTF(struct tf_param_ &tf_param){
    tf_param.transform.translation.x = tf_param.x;
    tf_param.transform.translation.y = tf_param.y;
    tf_param.transform.translation.z = tf_param.z;

    tf2::Quaternion tf_RM;
    tf_RM.setRPY(tf_param.roll * M_PI/180, tf_param.pitch * M_PI/180, tf_param.yaw * M_PI/180);
    
    tf_param.transform.rotation.x = tf_RM.x();
    tf_param.transform.rotation.y = tf_RM.y();
    tf_param.transform.rotation.z = tf_RM.z();
    tf_param.transform.rotation.w = tf_RM.w();
}


void JointStateCallback(struct tf_param_ *tf_param, const sensor_msgs::JointStateConstPtr& msg){
    // static ros::Time ros_time0 = ros::Time(0);
    // if(ros_time0 == ros::Time(0)) ros_time0 = ros::Time::now();
    // ros::Time ros_time = ros::Time::now();
    // ros::Duration ros_dt = ros_time - ros_time0;
    // float dt = ros_dt.toSec();

    // for(auto i=0; i<4; i++) tf_param[i].pitch += msg->velocity[i] * 180/M_PI * dt;

    for(auto i=0; i<4; i++){
        tf_param[i].pitch = msg->position[i] * 180/M_PI;
        UpdateTF(tf_param[i]);
    }

    // ros_time0 = ros_time;
}


void WheelMarkerInit(visualization_msgs::MarkerArray *Array){
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.ns = "wheel";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.25;
    marker.color.g = 0.25;
    marker.color.b = 0.25;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/meshes/wheel.dae";

    for(int i=0; i<4; i++){
        Array->markers.push_back(marker);
        Array->markers[i].id = i+1;
    }
    Array->markers[0].header.frame_id = "FL";
    Array->markers[1].header.frame_id = "FR";
    Array->markers[2].header.frame_id = "RL";
    Array->markers[3].header.frame_id = "RR";
}

void VehicleMarkerInit(visualization_msgs::MarkerArray *Array){
    static visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.ns = "husky";
    marker.header.frame_id = "vehicle_frame";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.z = 0.7071068;   // yaw 90
    // marker.pose.orientation.w = 0.7071068;
    marker.pose.position.z = 0.15;
    marker.pose.orientation.z = 0.0;    // yaw 90
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/meshes/base_link.dae";

    Array->markers.push_back(marker);
    Array->markers[4].id = 5;
}

void TopFrameMarkerInit(visualization_msgs::MarkerArray *Array){
    static visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.ns = "husky";
    marker.header.frame_id = "vehicle_frame";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.z = 0.7071068;   // yaw 90
    // marker.pose.orientation.w = 0.7071068;
    marker.pose.position.z = 0.15;
    marker.pose.orientation.z = 0.0;    // yaw 90
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.2;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/meshes/top_chassis.dae";

    Array->markers.push_back(marker);
    Array->markers[5].id = 6;
}

void RailFrameMarkerInit(visualization_msgs::MarkerArray *Array){
    static visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.ns = "husky";
    marker.header.frame_id = "vehicle_frame";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.z = 0.7071068;   // yaw 90
    // marker.pose.orientation.w = 0.7071068;
    marker.pose.position.x = 0.28;
    marker.pose.position.z = 0.39;
    marker.pose.orientation.z = 0.0;    // yaw 90
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/meshes/user_rail.dae";

    Array->markers.push_back(marker);
    Array->markers[6].id = 7;
}

void BumperMarkerInit(visualization_msgs::MarkerArray *Array){
    static visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time();
    marker.ns = "husky";
    marker.header.frame_id = "vehicle_frame";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.z = 0.7071068;   // yaw 90
    // marker.pose.orientation.w = 0.7071068;
    marker.pose.position.x = 0.48;
    marker.pose.position.z = 0.25;
    marker.pose.orientation.z = 0.0;    // yaw 90
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pharos_tf2/meshes/bumper.dae";

    Array->markers.push_back(marker);
    Array->markers[7].id = 8;

    marker.pose.position.x = -0.48;
    marker.pose.orientation.z = 1;
   marker.pose.orientation.w = 0;
    Array->markers.push_back(marker);
    Array->markers[8].id = 9;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "pharos_tf_broadcaster");

    ros::NodeHandle node;
    ros::NodeHandle pnode("~");

    int rate_;
    pnode.param("rate", rate_, 100);
    ros::Rate rate(rate_);

    int tf_num_;
    pnode.getParam("tf_num", tf_num_);
    tf_param_ *tf_param = new tf_param_[tf_num_];


    static tf2_ros::TransformBroadcaster br;
    static tf2_ros::StaticTransformBroadcaster static_br;

    ros::Subscriber JointState_sub = node.subscribe<sensor_msgs::JointState>("/joint_states", 1, 
        boost::bind(&JointStateCallback, boost::ref(tf_param), _1));

    // odom -> map static_tf publish //aft_mapped_to_init /lego_loam/odometry
    ros::Subscriber Map_Init_sub = node.subscribe<nav_msgs::Odometry>("/map/lego_loam", 1, 
        boost::bind(&Map_TF_Init, boost::ref(static_br), _1));

    ros::Subscriber LeGO_LOAM_Odom_sub = node.subscribe<nav_msgs::Odometry>("/map/lego_loam", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber Husky_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/odometry", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber Vehicle_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/vehicle", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber SBG_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/sbg", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber Ublox_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/ublox", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber EKF_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/ekf", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));
    ros::Subscriber SLAM_Odom_sub = node.subscribe<nav_msgs::Odometry>("/odom/slam", 1, 
        boost::bind(&Odometry2TF, boost::ref(br), _1));

    ros::Publisher Vehicle_pub = node.advertise<visualization_msgs::MarkerArray>( "/vehicle_marker", 0 );

    for(int i=0; i<tf_num_; i++){
        //loading tf_config.yaml
        std::stringstream tf_name;
        tf_name << "tf_" << i+1;
        pnode.getParam(tf_name.str()+"/frame_id", tf_param[i].frame_id);
        pnode.getParam(tf_name.str()+"/child_frame_id", tf_param[i].child_frame_id);
        pnode.getParam(tf_name.str()+"/x", tf_param[i].x);
        pnode.getParam(tf_name.str()+"/y", tf_param[i].y);
        pnode.getParam(tf_name.str()+"/z", tf_param[i].z);
        pnode.getParam(tf_name.str()+"/roll", tf_param[i].roll);
        pnode.getParam(tf_name.str()+"/pitch", tf_param[i].pitch);
        pnode.getParam(tf_name.str()+"/yaw", tf_param[i].yaw);

        UpdateTF(tf_param[i]);
    }

    visualization_msgs::MarkerArray vehicle_marker;
    WheelMarkerInit(&vehicle_marker);
    VehicleMarkerInit(&vehicle_marker);
    TopFrameMarkerInit(&vehicle_marker);
    RailFrameMarkerInit(&vehicle_marker);
    BumperMarkerInit(&vehicle_marker);


    for(int i=0; i<tf_num_; i++){
        tf_stamped_.header.stamp = ros::Time::now();
        tf_stamped_.header.frame_id = tf_param[i].frame_id;
        tf_stamped_.child_frame_id = tf_param[i].child_frame_id;
        tf_stamped_.transform = tf_param[i].transform;

        static_br.sendTransform(tf_stamped_);

    }

    while(ros::ok()){

        for(int i=0; i<4; i++){
            tf_stamped_.header.stamp = ros::Time::now();
            tf_stamped_.header.frame_id = tf_param[i].frame_id;
            tf_stamped_.child_frame_id = tf_param[i].child_frame_id;
            tf_stamped_.transform = tf_param[i].transform;

            static_br.sendTransform(tf_stamped_);
        }

        Vehicle_pub.publish(vehicle_marker);
        // FR_pub.publish(markerFR);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
