/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * for a da Vinci PSM with respect to a pick-up camera held by another PSM.
 * 
 * PSM2 (Green Arm) will hold the camera.
 * PSM1 (Yellow Arm) will hold the tool.
 *
 */

#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/Joy.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


geometry_msgs::Pose rec_msg_1, rec_msg_2;
std::vector<int> isCam, isCoag, isHead;
int camPressed, coagPressed, headPressed, switchPressed;
std::string MTMName;

void callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_msg_1 = msg->pose;

}

void callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_msg_2 = msg->pose;

}

void callbackCam(const sensor_msgs::Joy::ConstPtr& msg)
{
	isCam = msg->buttons;
	camPressed = isCam[0];

	ROS_INFO("Cam pressed: %d", camPressed);
}

void callbackCoag(const sensor_msgs::Joy::ConstPtr& msg)
{
	isCoag = msg->buttons;
	coagPressed = isCoag[0];

	ROS_INFO("Coag pressed: %d", coagPressed);	
}

void callbackHead(const sensor_msgs::Joy::ConstPtr& msg)
{
	isHead = msg->buttons;
	headPressed = isHead[0];

	ROS_INFO("Head pressed: %d", headPressed);
}

void callbackSwitch(const std_msgs::String::ConstPtr& msg)
{
	MTMName = msg->data;
	switchPressed = 1;
	ROS_INFO("Switching enabled for MTM: %s", MTMName.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "setPSM1Base");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber sub1 = n.subscribe("/dvrk/PSM1/position_cartesian_current", 1000, callbackPSM1);
  ros::Subscriber sub2 = n.subscribe("/dvrk/PSM2/position_cartesian_local_current", 1000, callbackPSM2);
  ros::Subscriber cam_sub = n.subscribe("/dvrk/footpedals/camera", 1000, callbackCam);
  ros::Subscriber coag_sub = n.subscribe("/dvrk/footpedals/coag", 1000, callbackCoag);
  ros::Subscriber head_sub = n.subscribe("/dvrk/footpedals/operatorpresent", 1000, callbackHead);
  ros::Subscriber switch_sub = n.subscribe("/switch", 1000, callbackSwitch);
  
  // publishers
  ros::Publisher transform_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_base_frame", 1000);
  ros::Publisher rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM1/set_registration_rotation", 1000);
  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  ros::Publisher MTMR_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM1/set_desired_state", 1000);
  ros::Publisher MTML_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM1/set_desired_state", 1000);
  ros::Publisher MTMR_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM2/set_desired_state", 1000);
  ros::Publisher switch_pub = n.advertise<std_msgs::String>("/switch", 1000);
  ros::Publisher teleop_switch_pub = n.advertise<diagnostic_msgs::KeyValue>("/dvrk/console/teleop/select_teleop_psm", 1000);

  ros::Rate loop_rate(10);

  int switchCount = 1; 
  int baseFrameCount = 0;
  int regRotFlag = 0;
  while (ros::ok())
  {
    
    //////////////////////////////////////////////////////////////////////////////////
    // Transforms                                                                   //
    // ----------                                                                   //
    // H_Base1_W      : Base 1 to World                                             //
    // H_Base2_W      : Base 2 to World                                             //
    // H_Base1_Base2  : Base 1 to Base 2                                            //
    // H_Tool1_Base1  : Tooltip 1 to Base 1                                         //
    // H_Tool2_Base2  : Tooltip 2 to Base 2                                         //
    // H_Base2_Tool2  : Base 2 to Tooltip 2                                         //
    // H_Tool2_Cam    : Tooltip 2 to Pick-Up Camera                                 //
    // H_Cam_Tool2    : Pick-Up Camera to Tooltip 2                                 //
    // H_Cam          : Final Transform - Base 1 to Pick-Up Camera                  //
    //////////////////////////////////////////////////////////////////////////////////

    // creating object to broadcast transforms 
    static tf2_ros::TransformBroadcaster br;

    // for H_Base1_Base2 - from MATLAB script
    geometry_msgs::Pose temp;
    temp.position.x = 0.0;
    temp.position.y = 0.0;
    temp.position.z = 0.0;
    temp.orientation.w = 1.0;
    temp.orientation.x = 0.0;
    temp.orientation.y = 0.0;
    temp.orientation.z = 0.0;
    
    tf::Pose H_Base1_Base2;
    tf::poseMsgToTF(temp, H_Base1_Base2);     
    

    // from solidworks model:
    // for H_Tool2_Cam
    geometry_msgs::Pose temp1;
    temp1.position.x = 0.0;
    temp1.position.y = 0.0;
    temp1.position.z = 0.0;
    temp1.orientation.w = 1; //0.7071;
    temp1.orientation.x = 0; //0.7071;
    temp1.orientation.y = 0.0;
    temp1.orientation.z = 0.0;
    
    tf::Pose H_Tool2_Cam;
    tf::poseMsgToTF(temp1, H_Tool2_Cam);

    // ROS_INFO("1: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Tool2_Cam.getOrigin().getX(), H_Tool2_Cam.getOrigin().getY(), H_Tool2_Cam.getOrigin().getZ(), H_Tool2_Cam.getRotation().w(), H_Tool2_Cam.getRotation().x(), H_Tool2_Cam.getRotation().y(), H_Tool2_Cam.getRotation().z());

    tf::Transform H_Cam_Tool2 = H_Tool2_Cam.inverse(); //(H_Tool2_Cam);
    // H_Cam_Tool2.inverse();
    // ROS_INFO("2: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Cam_Tool2.getOrigin().getX(), H_Cam_Tool2.getOrigin().getY(), H_Cam_Tool2.getOrigin().getZ(), H_Cam_Tool2.getRotation().w(), H_Cam_Tool2.getRotation().x(), H_Cam_Tool2.getRotation().y(), H_Cam_Tool2.getRotation().z());

    // broadcasting
    geometry_msgs::TransformStamped tf_H_Cam_Tool2;
  
    tf_H_Cam_Tool2.header.stamp = ros::Time::now();
    tf_H_Cam_Tool2.header.frame_id = "tool2";
    tf_H_Cam_Tool2.child_frame_id = "camera";
    tf_H_Cam_Tool2.transform.translation.x = H_Cam_Tool2.getOrigin().getX();
    tf_H_Cam_Tool2.transform.translation.y = H_Cam_Tool2.getOrigin().getY();
    tf_H_Cam_Tool2.transform.translation.z = H_Cam_Tool2.getOrigin().getZ();
    tf_H_Cam_Tool2.transform.rotation.w = H_Cam_Tool2.getRotation().w();
    tf_H_Cam_Tool2.transform.rotation.x = H_Cam_Tool2.getRotation().x();
    tf_H_Cam_Tool2.transform.rotation.y = H_Cam_Tool2.getRotation().y();
    tf_H_Cam_Tool2.transform.rotation.z = H_Cam_Tool2.getRotation().z();

    // using data retrieved from subscriber
    tf::Pose H_Tool2_Base2;
    tf::poseMsgToTF(rec_msg_2, H_Tool2_Base2);

    // // for non-moving da Vinci arm holding camera
    // geometry_msgs::Pose temp2;
    // temp2.position.x = ;
    // temp2.position.y = ;
    // temp2.position.z = ;
    // temp2.orientation.w = ;
    // temp2.orientation.x = ;
    // temp2.orientation.y = ;
    // temp2.orientation.z = ;
    // tf::Pose H_Tool2_Base2;
    // tf::poseMsgToTF(temp2, H_Tool2_Base2);

  
    tf::Transform H_Base2_Tool2 = H_Tool2_Base2.inverse();

    // RO2_INFO("2: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Base2_Tool2.getOrigi2().getX(), H2Base2_Tool2.getOrigi2().getY(), H2Base2_Tool2.getOrigi2().getZ(), H2Base2_Tool2.getRotat2on().w(), H_2ase2_Tool2.getRotat2on().x(), H_2ase2_Tool2.getRotat2on().y(), H_2ase2_Tool2.getRotat2on().z());
  

    // tf::Transform T1;
    // T1.mult(H_Base2_Tool2, H_Base12Base2);

    // tf::Transform H_Cam;
    // H_Cam.mult(H_Tool2_Cam, T1);


    // Tooltip 1 position (including base frame, ie., wrt camera. That's why subscribed topic is NOT local.)
    tf::Pose H_Tool1;
    tf::poseMsgToTF(rec_msg_1, H_Tool1);
	// broadcasting
    geometry_msgs::TransformStamped tf_H_Tool1;
  
    tf_H_Tool1.header.stamp = ros::Time::now();
    tf_H_Tool1.header.frame_id = "new_cam";
    tf_H_Tool1.child_frame_id = "new_tool1";
    tf_H_Tool1.transform.translation.x = H_Tool1.getOrigin().getX();
    tf_H_Tool1.transform.translation.y = H_Tool1.getOrigin().getY();
    tf_H_Tool1.transform.translation.z = H_Tool1.getOrigin().getZ();
    tf_H_Tool1.transform.rotation.w = H_Tool1.getRotation().w();
    tf_H_Tool1.transform.rotation.x = H_Tool1.getRotation().x();
    tf_H_Tool1.transform.rotation.y = H_Tool1.getRotation().y();
    tf_H_Tool1.transform.rotation.z = H_Tool1.getRotation().z();


    tf::Pose H_Cam = H_Tool2_Cam * H_Base2_Tool2 * H_Base1_Base2;
    // ROS_INFO("1: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Final.getOrigin().getX(), H_Final.getOrigin().getY(), H_Final.getOrigin().getZ(), H_Final.getRotation().w(), H_Final.getRotation().x(), H_Final.getRotation().y(), H_Final.getRotation().z());

    geometry_msgs::Pose msg;
    tf::poseTFToMsg(H_Cam, msg);

    // broadcasting
    tf::Transform H_Cam_Base1 = H_Cam.inverse(); 
    geometry_msgs::TransformStamped tf_H_Cam_Base1;
  
    tf_H_Cam_Base1.header.stamp = ros::Time::now();
    tf_H_Cam_Base1.header.frame_id = "base1";
    tf_H_Cam_Base1.child_frame_id = "new_cam";
    tf_H_Cam_Base1.transform.translation.x = H_Cam_Base1.getOrigin().getX();
    tf_H_Cam_Base1.transform.translation.y = H_Cam_Base1.getOrigin().getY();
    tf_H_Cam_Base1.transform.translation.z = H_Cam_Base1.getOrigin().getZ();
    tf_H_Cam_Base1.transform.rotation.w = H_Cam_Base1.getRotation().w();
    tf_H_Cam_Base1.transform.rotation.x = H_Cam_Base1.getRotation().x();
    tf_H_Cam_Base1.transform.rotation.y = H_Cam_Base1.getRotation().y();
    tf_H_Cam_Base1.transform.rotation.z = H_Cam_Base1.getRotation().z();


    // ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    if (baseFrameCount%10==0)
    	ROS_INFO("Updating base frame of PSM 1...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
    	baseFrameCount = 0;

    transform_pub.publish(msg);

    // sending transforms to visualize on rviz
    br.sendTransform(tf_H_Cam_Tool2);
    br.sendTransform(tf_H_Cam_Base1);
    br.sendTransform(tf_H_Tool1);

    // setting registration rotation of MTMR-PSM1
    geometry_msgs::Quaternion q;
    q.w = 0;
    q.x = 0;
    q.y = 1;
    q.z = 0;

    if(regRotFlag==0)
    {
    	ROS_INFO("Setting registration rotation for MTMR-PSM1");
    	regRotFlag = 1;
    }

    rot_pub.publish(q);


    // During teleoperation:
    // * Head sensor is required to enable teleoperation of both teleop pairs MTML-PSM2 (camera arm) and MTMR-PSM1 (tool arm)
    //		- Callback to subscribe to topic /dvrk/footpedals/operatorpresent.
    //		- The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates head sensed (1) or not (0).
    // * CAMERA footpedal determines which teleop pair is ACTIVELY enabled at the time
    //		- The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates footpedal pressed (1) or not (0).
    //		- When CAMERA footpedal is NOT pressed:
    // 			-- When headPressed = 1 (head sensed) AND camPressed = 0 (not pressed), enable MTMR-PSM1 (tool/non-camera arm) teleop by publishing to topic /dvrk
    //			-- Teleoperation between MTML-PSM2 will be frozen during this time.
    //		- When CAMERA footpedal is pressed:
    // 			-- When headPressed = 1 (head sensed) AND camPressed = 1 (pressed), enable MTML-PSM2 teleop by publishing to topic /dvrk
    //			-- Teleoperation between MTMR-PSM1 will be frozen during this time.
    

    std_msgs::String MTML_PSM2_State, MTMR_PSM1_State, MTML_PSM1_State, MTMR_PSM2_State;
    std::stringstream s1, s2, ss;

    // checking for presence of operator (head)
    if (headPressed == 1)
    {
    	// checking if CAMERA footpedal has been pressed
    	if (camPressed == 1)
    	{
    		// enabling MTML-PSM2 teleop // enabling pickup camera arm teleop
    		s1 << "ENABLED";
    		MTML_PSM2_State.data = s1.str();
    		MTMR_PSM2_State.data = s1.str();

    		// freezing MTMR-PSM1 teleop // freezing non-pickup camera arm teleop
    		s2 << "ALIGNING_MTM";
  	    	MTMR_PSM1_State.data = s2.str();
  	    	MTML_PSM1_State.data = s2.str();
    	}
    	else
    	{
    		// freezing MTML-PSM2 teleop // freezing pickup camera arm teleop
    		s1 << "ALIGNING_MTM";
    		MTML_PSM2_State.data = s1.str();
    		MTMR_PSM2_State.data = s1.str();

    		// enabling MTMR-PSM1 teleop // enabling non-pickup camera arm teleop
    		s2 << "ENABLED";
  	    	MTMR_PSM1_State.data = s2.str();
  	    	MTML_PSM1_State.data = s2.str();
    	}
    }
    else
    {
    	// freezing all teleops
    	ss << "ALIGNING_MTM";
    	MTMR_PSM1_State.data = ss.str();
    	MTML_PSM1_State.data = ss.str();
    	MTMR_PSM2_State.data = ss.str();
    	MTML_PSM2_State.data = ss.str();
    }

    // publishing to only active teleop pairs
    if (switchCount%2==1)
    {
    	MTMR_PSM1_pub.publish(MTMR_PSM1_State);
    	MTML_PSM2_pub.publish(MTML_PSM2_State);	
    }
    else
    {
    	MTMR_PSM2_pub.publish(MTMR_PSM2_State);
    	MTML_PSM1_pub.publish(MTML_PSM1_State);	
    }
    

	// Switching PSMs
	// * COAG footpedal is used to make the switch    
   	// * The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates footpedal pressed (1) or not (0).
   	// * Each time COAG is pressed, the teleop pairs must be switched. Use a count variable. 
   	// * For all odd counts, the original teleop pairs are used.
   	// * For all even counts, the reverse teleop pairs are used.
    
    
    diagnostic_msgs::KeyValue teleopPair;
    // switching teleop pairs when message is published to topic /switch
    if (switchPressed == 1)
    {	
    	switchPressed = 0;
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
        teleopPair.value = "PSM1";
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
        teleopPair.value = "PSM1";
        teleop_switch_pub.publish(teleopPair);
        // ROS_INFO("...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTML";
        teleopPair.value = "PSM2";
        teleop_switch_pub.publish(teleopPair);
        ROS_INFO("Switching has finished.");
        ros::Duration(1).sleep(); // sleep for a second  
      }
      switchCount++;
      ros::Duration(0.25).sleep();
    }

////////////////////////////////// old teleop code:
 //    if (camPressed == 1)
 //    {	
 //    	s1 << "ENABLED";
 //  	    MTML_PSM2_State.data = s1.str();
 //    	// ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());
 //    }
	// else
	// {
	// 	s1 << "ALIGNING_MTM";
	// 	MTML_PSM2_State.data = s1.str();
	// 	// ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());
	// }

	// MTML_PSM2_pub.publish(MTML_PSM2_State);


	// // coag footpedal - right - MTMR-PSM1 teleop
 //    if (coagPressed == 1)
 //    {
 //    	s2 << "ENABLED";
 //    	MTMR_PSM1_State.data = s2.str();
 //    	// ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());
 //    }
	// else
	// {
	// 	s2 << "ALIGNING_MTM";
	// 	MTMR_PSM1_State.data = s2.str();
	// 	// ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());
	// }

	// MTMR_PSM1_pub.publish(MTMR_PSM1_State);
//////////////////////////////////


    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}