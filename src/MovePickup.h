/**
 * This script moves the pickup camera to the desired position in cartesian space.
 */

#ifndef MOVEPICKUP_H
#define MOVEPICKUP_H

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//TF
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

class MovePickup
{
public:
	MovePickup();
	~MovePickup();

	void SetInputRotation();
	void run();

private:
	geometry_msgs::Pose ToolTip_msg, CamPos_msg, NewCamPos_msg, temp;
	
	tf::Transform ToolTip, CamPos, NewCamPos;
	tf::Transform H_ToolTip_Cam;
	
	tf::Quaternion InputRotation;

	ros::NodeHandle n;
	ros::Subscriber tooltip_sub;
	ros::Publisher goal_pub;

	//callbacks
	void callbackPSM(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void init();
	void MoveToPos();

};

#endif
