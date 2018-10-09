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
#include <sensor_msgs/JointState.h>

// TF
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

// Sockets
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#define SOCKET_ENABLED true

typedef struct sensor_pose
{
    float xpos;
    float ypos;
    float zpos;

    float qw;
    float qx;
    float qy;
    float qz;
}sensor_pose;

class MovePickup
{
public:
	MovePickup();
	~MovePickup();

	void SetInput();
	void run();

private:

	// flags
	bool newTopicInput;
	bool newSensorInput;
	bool jawSet;

	// ROS messages
	geometry_msgs::Pose ToolTipPos_msg, CamPos_msg, NewCamPos_msg, NewToolTipPos_msg;
	geometry_msgs::Pose TopicInputPose, SensorInputPose;
	double JawPosition;
	
	// Transforms
	tf::Transform ToolTipPos, CamPos, NewCamPos, NewToolTipPos;
	tf::Transform H_ToolTip_Cam, H_Cam_ToolTip;

	// ROS 
	ros::NodeHandle n;
	ros::Subscriber tooltip_sub, jaw_sub, input_sub;
	ros::Publisher goal_pub, jaw_pub, input_pub;

	//callbacks
	void callbackPSM(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void callbackJaw(const sensor_msgs::JointState::ConstPtr& msg);
	void callbackInputPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void init();
	void error(const char *msg);
	void MoveBy(geometry_msgs::Pose InputPose);

};

#endif
