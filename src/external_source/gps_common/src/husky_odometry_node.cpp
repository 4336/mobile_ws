/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <ublox_msgs/NavPVT.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgEkfQuat.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>

using namespace gps_common;

static ros::Publisher slam_odom_pub;
static ros::Publisher husky_odom_pub;
static ros::Publisher sbg_odom_pub;
static ros::Publisher ublox_odom_pub;
static ros::Publisher vehicle_frame_odom_pub;
static ros::Publisher vehicle_wgs_pub;

bool relative_gps_;
bool relative_gps_init_;
bool vehicle = false;

double origin_x_ = 0.0;
double origin_y_ = 0.0;
double origin_z_ = 0.0;
std::string zone_;

double heading_offset;

std::string frame_id, sbg_frame_id, ublox_frame_id, vehicle_frame;

nav_msgs::Odometry slam_odom_;
nav_msgs::Odometry husky_odom_;
nav_msgs::Odometry sbg_odom_;
nav_msgs::Odometry ublox_odom_;
nav_msgs::Odometry vehicle_frame_odom_;
nav_msgs::Odometry vehicle_odom_;
nav_msgs::Odometry vehicle_wgs_;

bool sbg_quat_init_ = false;
sbg_driver::SbgEkfQuat sbg_quat_data_;

double ublox_yaw = 0.0;
double antOffset = 0.0;

tf::Quaternion heading_quat_;


Eigen::Vector3f getRPYfromRM(Eigen::Matrix3f rotation_matrix){
        Eigen::Vector3f rpy;
        rpy[0] = atan2f(rotation_matrix(2,1),rotation_matrix(2,2));
        rpy[1] = atan2f(-rotation_matrix(2,0),sqrt(pow(rotation_matrix(2,1),2)+pow(rotation_matrix(2,2),2)));
        rpy[2] = atan2f(rotation_matrix(1,0),rotation_matrix(0,0));

        return rpy;
}

void Relative_gps_init(double origin_x, double origin_y, double origin_z){

    if(origin_x==0||origin_y==0||origin_z==0){
        ROS_ERROR("gps recived without fix");
        return;
    }
    
    bool relative_gps_fix_ = false;
    ros::param::get("~/relative_gps_fix", relative_gps_fix_);

    if( relative_gps_fix_ ){

        ros::param::get("~/fix_origin_x", origin_x);
        ros::param::get("~/fix_origin_y", origin_y);
        // ros::param::get("~/fix_origin_z", origin_z);
        printf("relative gps fix: true\nx: %f y: %f\n",origin_x,origin_y);

    }

    ros::param::set("gps/origin/x", origin_x);
    ros::param::set("gps/origin/y", origin_y);
    ros::param::set("gps/origin/z", origin_z);

    origin_x_ = origin_x;
    origin_y_ = origin_y;
    origin_z_ = origin_z;

    relative_gps_init_ = true;

    ROS_INFO("origin_x = %f, origin_y = %f, origin_z = %f",origin_x, origin_y, origin_z);

    return;
}

void SBG_Quat_callback(const sbg_driver::SbgEkfQuatConstPtr& data) {
    sbg_quat_init_ = true;
    sbg_quat_data_ = *data;
}

void SBG_Nav_callback(const sbg_driver::SbgEkfNavConstPtr& data) {
    if(!sbg_quat_init_) return;

    if (data->header.stamp == ros::Time(0)) {
        return;
    }

    double northing, easting;
    std::string zone;

    LLtoUTM(data->latitude, data->longitude, northing, easting, zone);

    if (sbg_odom_pub) {
        if(!relative_gps_init_) Relative_gps_init(easting, northing, data->altitude);

        sbg_odom_.header.stamp = data->header.stamp;

        if (frame_id.empty())
            sbg_odom_.header.frame_id = data->header.frame_id;
        else
            sbg_odom_.header.frame_id = frame_id;

        sbg_odom_.child_frame_id = sbg_frame_id;

        if(relative_gps_){
            sbg_odom_.pose.pose.position.x = easting - origin_x_;
            sbg_odom_.pose.pose.position.y = northing - origin_y_ ;
            sbg_odom_.pose.pose.position.z = data->altitude - origin_z_ ;
        }
        else{
            sbg_odom_.pose.pose.position.x = easting;
            sbg_odom_.pose.pose.position.y = northing;
            sbg_odom_.pose.pose.position.z = data->altitude;
        }

        sbg_odom_.pose.pose.orientation = sbg_quat_data_.quaternion;
        sbg_odom_pub.publish(sbg_odom_);
        if(vehicle_frame=="sbg_gps"){
            vehicle=true;
            vehicle_odom_ = sbg_odom_;
        }
    }
}


void Ublox_navpvt_callback(const ublox_msgs::NavPVTConstPtr& fix) {

    if (ublox_odom_pub) {
        double yaw;
        yaw = fix->heading;
        yaw = (yaw/100000.0-heading_offset)*M_PI/180.0;
        if(yaw < (M_PI/2.0)){
            yaw = yaw+(2*M_PI)-M_PI/2.0;
        }
        else{
            yaw -= M_PI/2.0;
        }
        yaw = 2*M_PI - yaw;
        heading_quat_.setRPY(0,0,yaw);
        ublox_yaw = yaw;
    }
}


void Ublox_fix_callback(const sensor_msgs::NavSatFixConstPtr& fix) {
    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_WARN("ublox no fix.");
        return;
    }
    if (fix->header.stamp == ros::Time(0)) {
        return;
    }

    double northing, easting;
    std::string zone;

    LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
    zone_ = zone;

    if (ublox_odom_pub) {

        if(!relative_gps_init_) Relative_gps_init(easting, northing, fix->altitude);


        double ublox_heading = 0.0;

        ublox_heading = (double)fix->position_covariance[1]; //unit: deg/1e-5
        ublox_heading = ublox_heading/100000.0;
        ublox_heading = ublox_heading*M_PI/180.0;

        if(ublox_heading < (M_PI/2.0)){

            ublox_heading = ublox_heading+(2*M_PI)-M_PI/2.0;

        }else{

            ublox_heading -= M_PI/2.0;

        }

        ublox_heading = 2*M_PI - ublox_heading;

//    geometry_msgs::Quaternion heading_quat;
//    heading_quat_.setRPY(0.0,0.0,ublox_heading);
//    tf::quaternionTFToMsg(heading_quat_,heading_quat);


        ublox_odom_.header.stamp = fix->header.stamp;

        if (frame_id.empty())
            ublox_odom_.header.frame_id = fix->header.frame_id;
        else
            ublox_odom_.header.frame_id = frame_id;


        ublox_odom_.child_frame_id = ublox_frame_id;


        if(relative_gps_){

            ublox_odom_.pose.pose.position.x = easting - origin_x_;
            ublox_odom_.pose.pose.position.y = northing - origin_y_ ;
            ublox_odom_.pose.pose.position.z = fix->altitude - origin_z_ ;

        }else{

            ublox_odom_.pose.pose.position.x = easting;
            ublox_odom_.pose.pose.position.y = northing;
            ublox_odom_.pose.pose.position.z = fix->altitude;

        }
        ublox_odom_.pose.pose.position.x -= antOffset*cos(ublox_yaw);
        ublox_odom_.pose.pose.position.y -= antOffset*sin(ublox_yaw);

        geometry_msgs::Quaternion heading_quat;
        tf::quaternionTFToMsg(heading_quat_,heading_quat);
        ublox_odom_.pose.pose.orientation = heading_quat;

        // Use ENU covariance to build XYZRPY covariance
        boost::array<double, 36> covariance = {{
            fix->position_covariance[0],
            fix->position_covariance[1],
            fix->position_covariance[2],
            0, 0, 0,
            fix->position_covariance[3],
            fix->position_covariance[4],
            fix->position_covariance[5],
            0, 0, 0,
            fix->position_covariance[6],
            fix->position_covariance[7],
            fix->position_covariance[7],
            0, 0, 0,
            0, 0, 0, fix->position_covariance[6], 0, 0,
            0, 0, 0, 0, fix->position_covariance[7], 0,
            0, 0, 0, 0, 0, fix->position_covariance[8]
        }};

        ublox_odom_.pose.covariance = covariance;
        ublox_odom_pub.publish(ublox_odom_);
        if(vehicle_frame == "ublox_gps"){
            vehicle=true;
            vehicle_odom_ = ublox_odom_;
        }
    }
}

void Mcl_callback(const nav_msgs::OdometryConstPtr& fix) {
    if(vehicle_frame == "mcl"){
        vehicle=true;
        vehicle_odom_ = *fix;
    }
}

void Ekf_callback(const nav_msgs::OdometryConstPtr& fix) {
    if(vehicle_frame == "ekf"){
        vehicle=true;
        vehicle_odom_ = *fix;
    }
    if(vehicle_frame == "ekf"){
        vehicle=true;
        vehicle_odom_ = *fix;
    }
    if(vehicle_frame == "ekf"){
        vehicle=true;
        vehicle_odom_ = *fix;
    }

    double Lat, Long;

    UTMtoLL(fix->pose.pose.position.y + origin_y_, fix->pose.pose.position.x + origin_x_, zone_, Lat, Long);

    vehicle_wgs_.pose.pose.position.x = Lat;
    vehicle_wgs_.pose.pose.position.y = Long;
    vehicle_wgs_.pose.pose.position.z = 0.0;

    vehicle_wgs_pub.publish(vehicle_wgs_);
}

void SLAM_Odometry_callback(tf2_ros::Buffer &tfBuffer, const nav_msgs::Odometry::ConstPtr& msg)
{
    static bool isInit = false;
    if(!isInit){
        isInit = true;
        return;
    }
    static geometry_msgs::TransformStamped tf_stamped;
    try{
      tf_stamped = tfBuffer.lookupTransform("odom", "lego_loam", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        // ros::Duration(0.1).sleep();
        return;
    }
    slam_odom_ = *msg;

    slam_odom_.header.frame_id = "odom";
    slam_odom_.child_frame_id = "slam";

    slam_odom_.pose.pose.position.x = tf_stamped.transform.translation.x;
    slam_odom_.pose.pose.position.y = tf_stamped.transform.translation.y;
    slam_odom_.pose.pose.position.z = tf_stamped.transform.translation.z;

    slam_odom_.pose.pose.orientation.x = tf_stamped.transform.rotation.x;
    slam_odom_.pose.pose.orientation.y = tf_stamped.transform.rotation.y;
    slam_odom_.pose.pose.orientation.z = tf_stamped.transform.rotation.z;
    slam_odom_.pose.pose.orientation.w = tf_stamped.transform.rotation.w;

    slam_odom_pub.publish(slam_odom_);

    if(vehicle_frame == "slam"){
        vehicle=true;
        vehicle_odom_ = slam_odom_;
    }
}

void Husky_Odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static geometry_msgs::Point p;
    static ros::Time ros_time0 = ros::Time(0);
    if(ros_time0 == ros::Time(0)){
        ros_time0 = ros::Time::now();

        p.x = 0;
        p.y = 0;
        p.z = 0;

        husky_odom_.header.frame_id = "odom";
        husky_odom_.child_frame_id = "odometry";
    }
    ros::Time ros_time = ros::Time::now();
    ros::Duration ros_dt = ros_time - ros_time0;
    float dt = ros_dt.toSec();
    // std::cout<<dt<<std::endl;

    p.x += cos(p.z) * msg->twist.twist.linear.x * dt;
    p.y += sin(p.z) * msg->twist.twist.linear.x * dt;
    p.z += msg->twist.twist.angular.z * dt;

    geometry_msgs::Quaternion msg_Q;
    msg_Q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, p.z);
    husky_odom_.pose.pose.position.x = p.x;
    husky_odom_.pose.pose.position.y = p.y;
    husky_odom_.pose.pose.orientation = msg_Q;

    husky_odom_.twist = msg->twist;

    husky_odom_.header.stamp = ros::Time::now();
    husky_odom_pub.publish(husky_odom_);
    if(vehicle_frame == "odometry"){
        vehicle=true;
        vehicle_odom_ = husky_odom_;
    }

    ros_time0 = ros_time;
}

int main (int argc, char **argv) {
    // Setup //

    ros::init(argc, argv, "pharos_utm_odometry_node");

    ros::NodeHandle node("");
    ros::NodeHandle pnode("~");

    node.param<std::string>("frame_id", frame_id, "odom");
    node.param<std::string>("vehicle_frame_id", vehicle_frame, "ublox_ekf");

    node.param<std::string>("sbg_frame_id", sbg_frame_id, "sbg_gps");
    node.param<std::string>("ublox_frame_id", ublox_frame_id, "ublox_gps");
    
    pnode.param<double>("antOffset", antOffset, 0.0);
    pnode.param("relative_gps", relative_gps_, true);
    pnode.param<double>("heading_offset", heading_offset, 0.0);

    slam_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/slam", 100);
    husky_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/odometry", 100);
    sbg_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/sbg", 100);
    ublox_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/ublox", 100);
    vehicle_frame_odom_pub = node.advertise<nav_msgs::Odometry>("/odom/vehicle", 100);
    vehicle_wgs_pub = node.advertise<nav_msgs::Odometry>("/wgs/vehicle", 100);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Subscriber SLAM_sub = node.subscribe<nav_msgs::Odometry>("/map/lego_loam", 1, 
        boost::bind(&SLAM_Odometry_callback, boost::ref(tfBuffer), _1));

    ros::Subscriber Husky_sub = node.subscribe("/husky_velocity_controller/odom", 10, Husky_Odom_callback);
    ros::Subscriber SbgNav_sub = node.subscribe("/sbg/ekf_nav", 10, SBG_Nav_callback);
    ros::Subscriber SbgQuat_sub = node.subscribe("/sbg/ekf_quat", 10, SBG_Quat_callback);

    ros::Subscriber fix_sub = node.subscribe("/ublox/fix", 10, Ublox_fix_callback);
    ros::Subscriber NavPVT_sub = node.subscribe("/ublox/navpvt", 10, Ublox_navpvt_callback);

    ros::Subscriber mcl_sub = node.subscribe("/odom/mcl", 10, Mcl_callback);
    ros::Subscriber ekf_sub = node.subscribe("/odom/ekf", 10, Ekf_callback);

    // Loop //

    int rate_;
    pnode.param("rate", rate_, 100);
    ros::Rate rate(rate_);

    while(ros::ok()){
        vehicle_frame_odom_ = vehicle_odom_;
        vehicle_frame_odom_.child_frame_id = "vehicle_frame";

        if(vehicle) vehicle_frame_odom_pub.publish(vehicle_frame_odom_);

        ros::spinOnce();
        rate.sleep();
    }

}