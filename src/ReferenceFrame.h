#ifndef REFERENCEFRAME_H
#define REFERENCEFRAME_H

#include <vector>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/Joy.h>

// TF
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class ReferenceFrame
{

public:
	ReferenceFrame();
	~ReferenceFrame();

	void run();

private:
	// PSM position data
	geometry_msgs::Pose rec_msg_1;
	geometry_msgs::Pose rec_msg_2;
	geometry_msgs::Pose rec_msg_3;

	// Console controls
	std::vector<int> isCam;
	std::vector<int> isHead;
	int camPressed;
	int headPressed;
	int switchPressed;

	// flags
	int switchFlag;
	int switchCount;
	int baseFrameCount;
	int regRotFlag;
	int rate;

	// ROS node handle
	ros::NodeHandle n;

	// ROS Subscribers
	ros::Subscriber sub1, sub2, sub3;
	ros::Subscriber cam_sub, head_sub, switch_sub;

	// ROS Publishers
	ros::Publisher psm2_pub, psm3_pub;
	ros::Publisher MTML_PSM2_rot_pub, MTMR_PSM3_rot_pub, MTMR_PSM2_rot_pub, MTML_PSM3_rot_pub;
	ros::Publisher MTML_PSM2_pub, MTMR_PSM3_pub, MTML_PSM3_pub, MTMR_PSM2_pub;
	ros::Publisher switch_pub;
	ros::Publisher teleop_switch_pub;

	// Transforms
	tf::Pose H_Base2_Base1, H_Base3_Base1;
	tf::Pose H_Tool1_Cam;
	tf::Pose H_Tool1_Base1;
	tf::Transform H_Base1_Tool1;
	tf::Pose H_Base2_Cam, H_Base3_Cam; 

	geometry_msgs::Pose msg2, msg3;

	geometry_msgs::Quaternion q;

	// TF
	static tf2_ros::TransformBroadcaster br;

	// Callback functions
	void callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackPSM3(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void callbackCam(const sensor_msgs::Joy::ConstPtr& msg);
	void callbackHead(const sensor_msgs::Joy::ConstPtr& msg);
	void callbackSwitch(const std_msgs::String::ConstPtr& msg);

	// Member functions
	void init();
	void setup();
	void broadcast();
	void switchTeleopPair();
	void setScale();


};

#endif