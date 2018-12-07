#ifndef DVRK_DATA_H
#define DVRK_DATA_H

#include <stdio.h>
// #include <fstream.h>

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// TF
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>


class DVRKData
{

public:
	DVRKData();
	~DVRKData();

	void startSubscribing();
	void run();

private:
	// PSM position data
	geometry_msgs::Pose psm1_tooltip_msg, psm2_tooltip_msg;
	geometry_msgs::Pose psm2_tooltip_in_psm1_msg;

	// ROS node handle
	ros::NodeHandle n;

	// ROS Subscribers
	ros::Subscriber psm1_sub, psm2_sub;
	ros::Subscriber save_sub;

	// ROS Publishers
	ros::Publisher save_pub;

	// Transforms
	tf::Pose PSM1_H_PSM2, PSM1_H_PSM1_TOOLTIP, PSM1_TOOLTIP_H_PSM1;
	tf::Pose psm1_tooltip, psm2_tooltip;
	tf::Point psm2_tooltip_in_psm1;

	int rate;
	bool save;
	int count;


	// Callback functions
	void callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackSave(const std_msgs::String::ConstPtr& msg);

	// Member functions
	void saveToFile();

};

#endif