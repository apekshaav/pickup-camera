/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of the two da Vinci PSMs (PSM2 and PSM3) with respect to a pick-up camera held by another PSM (PSM1).
 * 
 * PSM2 (Green Arm) will hold a tool.
 * PSM3 (Red Arm) will hold another tool.
 * PSM1 (Yellow Arm) will hold the pick-up camera.
 *
 */

#include "ReferenceFrame.h"

// flag to enable broadcasting transforms for display on rviz. Enable for debugging purposes.
#define BROADCAST_TRANSFORMS 0

ReferenceFrame::ReferenceFrame()
{
	// set loop rate
	rate = 10; //Hz

	// initializing flags
	switchFlag = 0;
  	switchCount = 1; 
  	baseFrameCount = 0;
  	regRotFlag = 0;

  	init();
}

ReferenceFrame::~ReferenceFrame()
{
	// clean up
}

// callbacks

void ReferenceFrame::callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	rec_msg_1 = msg->pose;
  	// ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", rec_msg_1.position.x, rec_msg_1.position.y, rec_msg_1.position.z, rec_msg_1.orientation.w, rec_msg_1.orientation.x, rec_msg_1.orientation.y, rec_msg_1.orientation.z);
}
	
void ReferenceFrame::callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	rec_msg_2 = msg->pose;
}

void ReferenceFrame::callbackPSM3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	rec_msg_3 = msg->pose;
}
	
void ReferenceFrame::callbackCam(const sensor_msgs::Joy::ConstPtr& msg)
{
	isCam = msg->buttons;
	camPressed = isCam[0];

	ROS_INFO("Cam pressed: %d", camPressed);
}

void ReferenceFrame::callbackHead(const sensor_msgs::Joy::ConstPtr& msg)
{
	isHead = msg->buttons;
	headPressed = isHead[0];

	ROS_INFO("Head pressed: %d", headPressed);
}
	
void ReferenceFrame::callbackSwitch(const std_msgs::String::ConstPtr& msg)
{	
	switchPressed = 1;
	ROS_INFO("Switching enabled: %s", msg->data.c_str());
}

void ReferenceFrame::init()
{
	// Starting ROS subscribers
	sub1 = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, &ReferenceFrame::callbackPSM1, this);

  	//cam_sub = n.subscribe("/dvrk/footpedals/camera", 1000, &ReferenceFrame::callbackCam, this);
  	//head_sub = n.subscribe("/dvrk/footpedals/operatorpresent", 1000, &ReferenceFrame::callbackHead, this);
  
  	switch_sub = n.subscribe("/pickup/switch", 1000, &ReferenceFrame::callbackSwitch, this);

  	// Starting ROS Publishers
  	psm2_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  	psm3_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM3/set_base_frame", 1000);
  
  	MTML_PSM2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  	MTMR_PSM3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM3/set_registration_rotation", 1000);

  	MTMR_PSM2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM2/set_registration_rotation", 1000);
  	MTML_PSM3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM3/set_registration_rotation", 1000);
  
  	//MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  	//MTMR_PSM3_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM3/set_desired_state", 1000);
  	//MTML_PSM3_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM3/set_desired_state", 1000);
  	//MTMR_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM2/set_desired_state", 1000);
  	
  	switch_pub = n.advertise<std_msgs::String>("/pickup/switch", 1000);
  	teleop_switch_pub = n.advertise<diagnostic_msgs::KeyValue>("/dvrk/console/teleop/select_teleop_psm", 1000);

	// for TF broadcasting
#if BROADCAST_TRANSFORMS
  	sub2 = n.subscribe("/dvrk/PSM2/position_cartesian_current", 1000, &ReferenceFrame::callbackPSM2, this); // we want position data, including base frame
  	sub3 = n.subscribe("/dvrk/PSM3/position_cartesian_current", 1000, &ReferenceFrame::callbackPSM3, this); // we want position data, including base frame
#endif

  	setup();

}
	
void ReferenceFrame::setup()
{
		
    //////////////////////////////////////////////////////////////////////////////////
    // Transforms                                                                   //
    // ----------                                                                   //
    // H_Base1_W      : Base 1 to World                                             //
    // H_Base2_W      : Base 2 to World                                             //
    // H_Base3_W      : Base 3 to World                                             //
    // H_Base2_Base1  : Base 2 to Base 1                                            //
    // H_Base3_Base1  : Base 3 to Base 1                                            //
    // H_Tool1_Base1  : Tooltip 1 to Base 1                                         //
    // H_Tool2_Base2  : Tooltip 2 to Base 2                                         //
    // H_Base1_Tool1  : Base 1 to Tooltip 1                                         //
    // H_Tool1_Cam    : Tooltip 1 to Pick-Up Camera                                 //
    // H_Cam_Tool1    : Pick-Up Camera to Tooltip 1                                 //
    // H_Base2_Cam    : Final Transform - Base 2 to Pick-Up Camera                  //
    // H_Base3_Cam    : Final Transform - Base 3 to Pick-Up Camera                  //
    //////////////////////////////////////////////////////////////////////////////////

	
	// Setting constant transforms-----------------------

	// for H_Base2_Base1 - from MATLAB script
    geometry_msgs::Pose temp;
    temp.position.x = 0.0081;
    temp.position.y = 0.2767;
    temp.position.z = -0.0580;
    temp.orientation.w = 0.5645;
    temp.orientation.x = -0.0469;
    temp.orientation.y = -0.0250;
    temp.orientation.z = 0.8237;
    tf::poseMsgToTF(temp, H_Base2_Base1);


    // for H_Base3_Base1 - from MATLAB script
    geometry_msgs::Pose temp1;
    temp1.position.x = -0.1779;
    temp1.position.y = 0.1433;
    temp1.position.z = 0.2481;
    temp1.orientation.w = 0.9702;
    temp1.orientation.x = 0.0076;
    temp1.orientation.y = -0.0296;
    temp1.orientation.z = -0.2402;
    tf::poseMsgToTF(temp1, H_Base3_Base1);


    // from solidworks model:
    // for H_Tool1_Cam
    geometry_msgs::Pose temp2;
    temp2.position.x = 0.035; //0.02645;
    temp2.position.y = 0.0;
    temp2.position.z = 0.0;
    temp2.orientation.w = 0.9659;
    temp2.orientation.x = -0.2588;
    temp2.orientation.y = 0.0;
    temp2.orientation.z = 0.0;
    tf::poseMsgToTF(temp2, H_Tool1_Cam);


    // setting registration rotations
    q.w = 0;
    q.x = 1;
    q.y = 0;
    q.z = 0;

}
	
void ReferenceFrame::broadcast()
{

}
	
void ReferenceFrame::switchTeleopPair()
{
	diagnostic_msgs::KeyValue teleopPair;

	if (switchCount%2==1)
    {
        teleopPair.key = "MTML";
        teleopPair.value = "";
        teleop_switch_pub.publish(teleopPair);
        ROS_INFO("Switching has begun...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTMR";
        teleopPair.value = "PSM2";
        teleop_switch_pub.publish(teleopPair);
        // ROS_INFO("...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTML";
        teleopPair.value = "PSM3";
        teleop_switch_pub.publish(teleopPair);
        ROS_INFO("Switching has finished.");
        ros::Duration(1).sleep(); // sleep for a second  
    }
    else
    {
        teleopPair.key = "MTML";
        teleopPair.value = "";
        teleop_switch_pub.publish(teleopPair);
        ROS_INFO("Switching has begun...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTMR";
        teleopPair.value = "PSM3";
        teleop_switch_pub.publish(teleopPair);
        // ROS_INFO("...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTML";
        teleopPair.value = "PSM2";
        teleop_switch_pub.publish(teleopPair);
        ROS_INFO("Switching has finished.");
        ros::Duration(1).sleep(); // sleep for a second  
    }

}
	
void ReferenceFrame::setScale()
{

}

void ReferenceFrame::run()
{
	ros::Rate loop_rate(rate);

	// where all the work happens

	while(ros::ok())
	{

		// get tooltip 1 position retrieved from subscriber callbackPSM1
    	tf::poseMsgToTF(rec_msg_1, H_Tool1_Base1);
		H_Base1_Tool1 = H_Tool1_Base1.inverse();

    	// transformation chains
	    H_Base2_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base2_Base1;
    	H_Base3_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base3_Base1;

    	// for PSM2
    	tf::poseTFToMsg(H_Base2_Cam, msg2);

    	// for PSM3
    	tf::poseTFToMsg(H_Base3_Cam, msg3);

    	// setting base frames
    	psm2_pub.publish(msg2);
    	psm3_pub.publish(msg3);
    	// ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg3.position.x, msg3.position.y, msg3.position.z, msg3.orientation.w, msg3.orientation.x, msg3.orientation.y, msg3.orientation.z);

    	if (baseFrameCount%10==0)
    		ROS_INFO("Updating base frames of PSM 2 and PSM3...");
    	baseFrameCount++;

    	//clearing and resetting every 1000 times
    	if (baseFrameCount==1000)
    		baseFrameCount = 0;

    	if(switchCount%2==1)
    	{
	    	MTML_PSM2_rot_pub.publish(q);
    		MTMR_PSM3_rot_pub.publish(q);
    	}
    	else
    	{
    		MTMR_PSM2_rot_pub.publish(q);
    		MTML_PSM3_rot_pub.publish(q);
    	}

    	if(regRotFlag==0)
    	{
        	ROS_INFO("Setting registration rotation for both teleop pairs");
        	regRotFlag = 1;
    	}


    	// setting initial teleop pairs
    	std_msgs::String switchString;
    	std::stringstream stemp;
    	if (switchFlag==0)
    	{
    		ROS_INFO("Initial switch");
    		ros::Duration(1).sleep();
    		switchTeleopPair();
	        stemp << "yes";
        	switchString.data = stemp.str();
        	// switch_pub.publish(switchString);
        	switchFlag = 1;
        	// ros::Duration(3.30).sleep();
        	// switchCount++;
    	}

    	// switching teleop pairs when message is published to topic /pickup/switch
    	if (switchPressed == 1)
    	{	
    		switchPressed = 0;
    		switchTeleopPair();
      		
      		switchCount++;
      		ros::Duration(0.3).sleep();
    	}


#if BROADCAST_TRANSFORMS
    	broadcast();
#endif

    	ros::spinOnce();
    	loop_rate.sleep();

	}
}
