/**
 * This script moves the pickup camera to the desired position in cartesian space.
 */

#include "MovePickup.h"

MovePickup::MovePickup()
{
	init();
}

MovePickup::~MovePickup()
{

}

void MovePickup::callbackPSM(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	ToolTip_msg = msg->pose;
}

void MovePickup::init()
{
    ROS_INFO("Init");

  	// subscribers
 	tooltip_sub = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, &MovePickup::callbackPSM, this);

 	// publishers
 	goal_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_position_cartesian", 1000);

 	// camera frame, H_cam
    geometry_msgs::Pose temp1;
    temp1.position.x = 0.035;
    temp1.position.y = 0.0;
    temp1.position.z = 0.0;
    temp1.orientation.w = 0.9659; 
    temp1.orientation.x = -0.2588; 
    temp1.orientation.y = 0.0;
    temp1.orientation.z = 0.0;
    tf::poseMsgToTF(temp1, H_Cam);
}

void MovePickup::SetInputRotation()
{
	tf::Quaternion InputRotation;
    InputRotation = tf::createQuaternionFromRPY(0.17, 0, 0); // in radians
    InputRotation.normalize();
}

void MovePickup::MoveToPos()
{

	// update camera position
    tf::poseMsgToTF(ToolTip_msg, ToolTip);

    // ROS_DEBUG("Position: [%f %f %f] ", ToolTip.getOrigin().x, ToolTip.getOrigin().y, ToolTip.getOrigin().z);
	
    CamPos = H_Cam * ToolTip;
    tf::poseTFToMsg(CamPos, NewCamPos_msg);

    // extract quaternion component
    tf::Quaternion q_old;
    q_old = CamPos.getRotation();

    // multiply quaternions
    tf::Quaternion q_new;
    q_new = InputRotation * q_old;
	q_new.normalize();
    quaternionTFToMsg(q_new, NewCamPos_msg.orientation);

    // move to new position
    //goal_pub.publish(NewCamPos_msg);
    ROS_INFO("New position updated");
    ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", NewCamPos_msg.position.x, NewCamPos_msg.position.y, NewCamPos_msg.position.z, NewCamPos_msg.orientation.w, NewCamPos_msg.orientation.x, NewCamPos_msg.orientation.y, NewCamPos_msg.orientation.z);

}

void MovePickup::run()
{
	ros::Rate loop_rate(100);

	while(ros::ok())
	{

        // poll callbacks
        ros::spinOnce();

		SetInputRotation();
		MoveToPos();

		loop_rate.sleep();
	}

}

