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
  	ToolTipPos_msg = msg->pose;
}

void MovePickup::init()
{
    ROS_INFO("Init");

  	// subscribers
 	tooltip_sub = n.subscribe("/dvrk/PSM1/position_cartesian_current", 1000, &MovePickup::callbackPSM, this);

 	// publishers
 	goal_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_position_cartesian", 1000);

 	// camera frame, H_ToolTip_Cam
    geometry_msgs::Pose temp1;
    temp1.position.x = 0.035;
    temp1.position.y = 0.0;
    temp1.position.z = 0.0;
    temp1.orientation.w = 1; //0.9659; 
    temp1.orientation.x = 0; //-0.2588; 
    temp1.orientation.y = 0.0;
    temp1.orientation.z = 0.0;
    tf::poseMsgToTF(temp1, H_ToolTip_Cam);

    H_Cam_ToolTip = H_ToolTip_Cam.inverse();
}

void MovePickup::SetInputRotation()
{
    InputRotation = tf::createQuaternionFromRPY(0, 0, 0.17); // in radians
    InputRotation.normalize();
    // geometry_msgs::Pose temp;
    // quaternionTFToMsg(InputRotation, temp.orientation);
    // ROS_INFO("SetInput: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp.position.x, temp.position.y, temp.position.z, temp.orientation.w, temp.orientation.x, temp.orientation.y, temp.orientation.z);
}

void MovePickup::MoveToPos()
{

	// update tooltip position
    tf::poseMsgToTF(ToolTipPos_msg, ToolTipPos);
    ROS_INFO("Tooltip: Position: [%f %f %f] Orientation: [%f %f %f %f]", ToolTipPos_msg.position.x, ToolTipPos_msg.position.y, ToolTipPos_msg.position.z, ToolTipPos_msg.orientation.w, ToolTipPos_msg.orientation.x, ToolTipPos_msg.orientation.y, ToolTipPos_msg.orientation.z);
	
    // get camera position
    CamPos = H_ToolTip_Cam * ToolTipPos;
    // tf::poseTFToMsg(CamPos, CamPos_msg);
    // ROS_INFO("CamPos: Position: [%f %f %f] Orientation: [%f %f %f %f]", CamPos_msg.position.x, CamPos_msg.position.y, CamPos_msg.position.z, CamPos_msg.orientation.w, CamPos_msg.orientation.x, CamPos_msg.orientation.y, CamPos_msg.orientation.z);

    // for rotation----
    #if 0
    // extract quaternion component
    tf::Quaternion q_old;
    q_old = CamPos.getRotation();
    // quaternionTFToMsg(q_old, temp.orientation);
    // ROS_INFO("Input: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp.position.x, temp.position.y, temp.position.z, temp.orientation.w, temp.orientation.x, temp.orientation.y, temp.orientation.z);

    // multiply quaternions
    tf::Quaternion q_new;
    q_new = InputRotation * q_old;
	q_new.normalize();    
    // geometry_msgs::Pose temp;
    // quaternionTFToMsg(q_new, temp.orientation);
    // ROS_INFO("new: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp.position.x, temp.position.y, temp.position.z, temp.orientation.w, temp.orientation.x, temp.orientation.y, temp.orientation.z);

    // setting new position
    NewCamPos.setRotation(q_new);
    //quaternionTFToMsg(q_new, NewCamPos_msg.orientation);
    #endif
    
    // for translation----
    #if 1
    tf::poseTFToMsg(CamPos, NewCamPos_msg);
    NewCamPos_msg.position.x = NewCamPos_msg.position.x + 0.0;
    NewCamPos_msg.position.y = NewCamPos_msg.position.y + 0.0;
    NewCamPos_msg.position.z = NewCamPos_msg.position.z - 0.01;
    tf::poseMsgToTF(NewCamPos_msg, NewCamPos);
    #endif

    //convert back into tooltip frame
    NewToolTipPos = H_Cam_ToolTip * NewCamPos;
    tf::poseTFToMsg(NewToolTipPos, NewToolTipPos_msg);

    // move to new position
    goal_pub.publish(NewToolTipPos_msg);
    ROS_INFO("New position updated");
    ROS_INFO("NewToolTipPos: Position: [%f %f %f] Orientation: [%f %f %f %f]", NewToolTipPos_msg.position.x, NewToolTipPos_msg.position.y, NewToolTipPos_msg.position.z, NewToolTipPos_msg.orientation.w, NewToolTipPos_msg.orientation.x, NewToolTipPos_msg.orientation.y, NewToolTipPos_msg.orientation.z);

}

void MovePickup::run()
{
	// ros::Rate loop_rate(100);

	//while(ros::ok())
	//{

        // poll callbacks
        ros::spinOnce();

		SetInputRotation();
		MoveToPos();

	//	loop_rate.sleep();
	//}

}

