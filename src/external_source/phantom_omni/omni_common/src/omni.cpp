#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include <pthread.h>

float prev_time;
int calibrationStyle;

struct OmniState {
	hduVector3Dd position;  //3x1 vector of position
	hduVector3Dd velocity;  //3x1 vector of velocity
	hduVector3Dd inp_vel1; //3x1 history of velocity used for filtering velocity estimate
	hduVector3Dd inp_vel2;
	hduVector3Dd inp_vel3;
	hduVector3Dd out_vel1;
	hduVector3Dd out_vel2;
	hduVector3Dd out_vel3;
	hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
	hduVector3Dd pos_hist2;
	hduVector3Dd rot;
	hduVector3Dd joints;
	hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
	float thetas[7];
	int buttons[2];
	int buttons_prev[2];
	bool lock;
	hduVector3Dd lock_pos;
};

class PhantomROS {

public:
	ros::NodeHandle n;
	ros::Publisher pose_publisher;
	ros::Publisher joint_pub;

	ros::Publisher tipOdom_publisher;
	ros::Publisher state_publisher;

	ros::Publisher button_publisher;
	ros::Subscriber haptic_sub;
	std::string omni_name;
	std::string sensable_frame_name;
	std::string link_names[7];

	OmniState *state;
	tf::TransformBroadcaster br; //WARN disabled by YJ

	void init(OmniState *s) {
		ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));

		//Publish pose on NAME/pose
		std::ostringstream stream00;
		stream00 << omni_name << "/pose";
		std::string pose_topic_name = std::string(stream00.str());
		pose_publisher = n.advertise<geometry_msgs::PoseStamped>(
				pose_topic_name.c_str(), 100);

		//Publish each joint state(rad) on joint_states
		joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

		//Publish tip odom(EE pose) on odom/tip
		tipOdom_publisher = n.advertise<nav_msgs::Odometry>("/odom/tip", 1);

		//Publish button state on NAME/button
		std::ostringstream stream0;
		stream0 << omni_name << "/button";
		std::string button_topic = std::string(stream0.str());
		button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(
				button_topic.c_str(), 100);

		//Subscribe force feedback to NAME/force_feedback
		std::ostringstream stream01;
		stream01 << "force_feedback";
		std::string force_feedback_topic = std::string(stream01.str());
		haptic_sub = n.subscribe(force_feedback_topic.c_str(), 100, &PhantomROS::force_callback, this);

		//Frame of force feedback (NAME_sensable)
		std::ostringstream stream2;
		stream2 << omni_name << "_sensable";
		sensable_frame_name = std::string(stream2.str());

		for (int i = 0; i < 7; i++) {
			std::ostringstream stream1;
			stream1 << omni_name << "_link" << i;
			link_names[i] = std::string(stream1.str());
		}

		state = s;
		state->buttons[0] = 0;
		state->buttons[1] = 0;
		state->buttons_prev[0] = 0;
		state->buttons_prev[1] = 0;
		hduVector3Dd zeros(0, 0, 0);
		state->velocity = zeros;
		state->inp_vel1 = zeros;  //3x1 history of velocity
		state->inp_vel2 = zeros;  //3x1 history of velocity
		state->inp_vel3 = zeros;  //3x1 history of velocity
		state->out_vel1 = zeros;  //3x1 history of velocity
		state->out_vel2 = zeros;  //3x1 history of velocity
		state->out_vel3 = zeros;  //3x1 history of velocity
		state->pos_hist1 = zeros; //3x1 history of position
		state->pos_hist2 = zeros; //3x1 history of position
		state->lock = true;
		state->lock_pos = zeros;

	}

	/*******************************************************************************
	 ROS node callback.
	 *******************************************************************************/
	void force_callback(const omni_msgs::OmniFeedbackConstPtr& omnifeed) {
		////////////////////Some people might not like this extra damping, but it
		////////////////////helps to stabilize the overall force feedback. It isn't
		////////////////////like we are getting direct impedance matching from the
		////////////////////omni anyway
		state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
		state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
		state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

		state->lock_pos[0] = omnifeed->position.x;
		state->lock_pos[1] = omnifeed->position.y;
		state->lock_pos[2] = omnifeed->position.z;
	}

	void publish_omni_state() {
		//Joint State
		//And Published it!!!
		sensor_msgs::JointState joint_state;
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position.resize(6);
		joint_state.name[0] = "waist";
		joint_state.position[0] = -state->thetas[1];
		joint_state.name[1] = "shoulder";
		joint_state.position[1] = state->thetas[2];
		joint_state.name[2] = "elbow";
		joint_state.position[2] = state->thetas[3];
		joint_state.name[3] = "yaw";
		joint_state.position[3] = -state->thetas[4] + M_PI;
		joint_state.name[4] = "pitch";
		joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
		joint_state.name[5] = "roll";
		joint_state.position[5] = state->thetas[6] - M_PI;
		joint_pub.publish(joint_state);


	//Tip pen(EE) position and orentation
	//And Published it!!!
		static tf::TransformListener listener;
		tf::StampedTransform transform;

		try{
			listener.lookupTransform("/base_haptic", "/tip",  
															 ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	
		static nav_msgs::Odometry tipOdom;
		tipOdom.header.stamp = ros::Time::now();
		tipOdom.header.frame_id = "base_haptic";
		tipOdom.pose.pose.position.x = transform.getOrigin().x();
		tipOdom.pose.pose.position.y = transform.getOrigin().y();
		tipOdom.pose.pose.position.z = transform.getOrigin().z();
		tipOdom.pose.pose.orientation.x = transform.getRotation().x();
		tipOdom.pose.pose.orientation.y = transform.getRotation().y();
		tipOdom.pose.pose.orientation.z = transform.getRotation().z();
		tipOdom.pose.pose.orientation.w = transform.getRotation().w();
		tipOdom_publisher.publish(tipOdom);


		//Sample 'end effector' pose
		//And Published it!!!
		geometry_msgs::PoseStamped pose_stamped;
		// pose_stamped.header.stamp = ros::Time::now();
		// pose_stamped.header.frame_id = link_names[6].c_str();
		pose_stamped.header = tipOdom.header;
		pose_stamped.pose = tipOdom.pose.pose;
		pose_publisher.publish(pose_stamped);


		//Button State
		//And Published it!!!
		if ((state->buttons[0] != state->buttons_prev[0])
				or (state->buttons[1] != state->buttons_prev[1])) {

			if ((state->buttons[0] == state->buttons[1])
					and (state->buttons[0] == 1)) {
				state->lock = !(state->lock);
			}
			omni_msgs::OmniButtonEvent button_event;
			button_event.grey_button = state->buttons[0];
			button_event.white_button = state->buttons[1];
			state->buttons_prev[0] = state->buttons[0];
			state->buttons_prev[1] = state->buttons[1];
			button_publisher.publish(button_event);
		}
	}
};// PhantomROS Class End




HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
	OmniState *omni_state = static_cast<OmniState *>(pUserData);
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
		ROS_DEBUG("Updating calibration...");
			hdUpdateCalibration(calibrationStyle);
		}
	hdBeginFrame(hdGetCurrentDevice());
	//Get angles, set forces
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
	hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);

	hduVector3Dd vel_buff(0, 0, 0);
	vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
			+ omni_state->pos_hist2) / 0.002;  //mm/s, 2nd order backward dif
	omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
			+ .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
			- (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
					- 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
	omni_state->pos_hist2 = omni_state->pos_hist1;
	omni_state->pos_hist1 = omni_state->position;
	omni_state->inp_vel3 = omni_state->inp_vel2;
	omni_state->inp_vel2 = omni_state->inp_vel1;
	omni_state->inp_vel1 = vel_buff;
	omni_state->out_vel3 = omni_state->out_vel2;
	omni_state->out_vel2 = omni_state->out_vel1;
	omni_state->out_vel1 = omni_state->velocity;
	if (omni_state->lock == true) {
		omni_state->force = 0.05 * (omni_state->lock_pos - omni_state->position)
				- 0.001 * omni_state->velocity;
	}

	hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

	//Get buttons
	int nButtons = 0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
	omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during main scheduler callback");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}

	double t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
			omni_state->joints[2] - omni_state->joints[1], omni_state->rot[0],
			omni_state->rot[1], omni_state->rot[2] };
	for (int i = 0; i < 7; i++)
		omni_state->thetas[i] = t[i];
	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
	int supportedCalibrationStyles;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
	}
	//ROS_INFO("Done1");
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..");
	}
	//ROS_INFO("Done11");
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
		calibrationStyle = HD_CALIBRATION_AUTO;
		ROS_INFO("HD_CALIBRATION_AUTO..");
	}
	//ROS_INFO("Done111");
	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
		do {
			//ROS_INFO("Done1111");
			hdUpdateCalibration(calibrationStyle);
			ROS_INFO("Calibrating.. (put tip in well)");
			if (HD_DEVICE_ERROR(error = hdGetError())) {
				hduPrintError(stderr, &error, "Reset encoders reset failed.");
				break;
			}
		} while (hdCheckCalibration() != HD_CALIBRATION_OK);
		ROS_INFO("Calibration complete.");
		} 
	//ROS_INFO("Done");
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
		ROS_INFO("Please place the device into the inkwell for calibration.");
	}
}

void *ros_publish(void *ptr) {
	//std::cout << "0";
	PhantomROS *omni_ros = (PhantomROS *) ptr;
	//std::cout << "1";
	int publish_rate;
	//std::cout << "3";
	omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
	//std::cout << "4";
	ros::Rate loop_rate(publish_rate);
	//std::cout << "5";
	ros::AsyncSpinner spinner(2);
	//std::cout << "6";
	spinner.start();
	//std::cout << "7";

	while (ros::ok()) {
		//std::cout << "8";
		ROS_INFO("Running");
		omni_ros->publish_omni_state();
		loop_rate.sleep();
	}
	return NULL;
}

int main(int argc, char** argv) {
	////////////////////////////////////////////////////////////////
	// Init Phantom
	////////////////////////////////////////////////////////////////
	HDErrorInfo error;
	HHD hHD;
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
		return -1;
	}

	ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		ROS_ERROR("Failed to start the scheduler"); //, &error);
		return -1;
	}
	HHD_Auto_Calibration();

	////////////////////////////////////////////////////////////////
	// Init ROS
	////////////////////////////////////////////////////////////////
	ros::init(argc, argv, "omni_haptic_node");
	OmniState state;
	PhantomROS omni_ros;

	omni_ros.init(&state);
	hdScheduleAsynchronous(omni_state_callback, &state,
			HD_MAX_SCHEDULER_PRIORITY);
	//ROS_INFO("Done1");

	////////////////////////////////////////////////////////////////
	// Loop and publish
	////////////////////////////////////////////////////////////////
	pthread_t publish_thread;
	//ROS_INFO("Done2");
	pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
	//ROS_INFO("Done3");
	pthread_join(publish_thread, NULL);
	//ROS_INFO("Done4");

	ROS_INFO("Ending Session....");
	hdStopScheduler();
	hdDisableDevice(hHD);

	return 0;
}
