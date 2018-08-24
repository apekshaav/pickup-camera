/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of the two da Vinci PSMs (PSM2 and PSM3) with respect to a pick-up camera held by another PSM (PSM1).
 * 
 * PSM2 (Green Arm) will hold a tool.
 * PSM3 (Red Arm) will hold another tool.
 * PSM1 (Yellow Arm) will hold the pick-up camera.
 *
 */

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


geometry_msgs::Pose rec_msg_1, rec_msg_2, rec_msg_3;
std::vector<int> isCam, isHead;
int camPressed, headPressed, switchPressed;
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

void callbackPSM3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_msg_3 = msg->pose;

}

void callbackCam(const sensor_msgs::Joy::ConstPtr& msg)
{
	isCam = msg->buttons;
	camPressed = isCam[0];

	ROS_INFO("Cam pressed: %d", camPressed);
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
  
  ros::init(argc, argv, "setPickup");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber sub2 = n.subscribe("/dvrk/PSM2/position_cartesian_current", 1000, callbackPSM2); // we want position data, including base frame
  ros::Subscriber sub3 = n.subscribe("/dvrk/PSM3/position_cartesian_current", 1000, callbackPSM3); // we want position data, including base frame
  
  ros::Subscriber sub1 = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, callbackPSM1);

  ros::Subscriber cam_sub = n.subscribe("/dvrk/footpedals/camera", 1000, callbackCam);
  ros::Subscriber head_sub = n.subscribe("/dvrk/footpedals/operatorpresent", 1000, callbackHead);

  ros::Subscriber switch_sub = n.subscribe("/switch", 1000, callbackSwitch);
  
  // publishers
  ros::Publisher psm2_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  ros::Publisher psm3_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM3/set_base_frame", 1000);
  
  ros::Publisher psm2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  ros::Publisher psm3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM3/set_registration_rotation", 1000);
  
  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  ros::Publisher MTMR_PSM3_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM3/set_desired_state", 1000);
  ros::Publisher MTML_PSM3_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM3/set_desired_state", 1000);
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

    // creating object to broadcast transforms 
    static tf2_ros::TransformBroadcaster br;

    // for H_Base2_Base1 - from MATLAB script
    geometry_msgs::Pose temp;
    temp.position.x = ;
    temp.position.y = ;
    temp.position.z = ;
    temp.orientation.w = ;
    temp.orientation.x = ;
    temp.orientation.y = ;
    temp.orientation.z = ;

    tf::Pose H_Base2_Base1;
    tf::poseMsgToTF(temp, H_Base2_Base1);

    // for H_Base3_Base1 - from MATLAB script
    geometry_msgs::Pose temp1;
    temp1.position.x = ;
    temp1.position.y = ;
    temp1.position.z = ;
    temp1.orientation.w = ;
    temp1.orientation.x = ;
    temp1.orientation.y = ;
    temp1.orientation.z = ;

    tf::Pose H_Base3_Base1;
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
    
    tf::Pose H_Tool1_Cam;
    tf::poseMsgToTF(temp2, H_Tool1_Cam);

    // for displaying in rviz, take the inverse
    tf::Transform H_Cam_Tool1 = H_Tool1_Cam.inverse();
    geometry_msgs::TransformStamped tf_H_Cam_Tool1;
  
    tf_H_Cam_Tool1.header.stamp = ros::Time::now();
    tf_H_Cam_Tool1.header.frame_id = "tool1";
    tf_H_Cam_Tool1.child_frame_id = "camera";
    tf_H_Cam_Tool1.transform.translation.x = H_Cam_Tool1.getOrigin().getX();
    tf_H_Cam_Tool1.transform.translation.y = H_Cam_Tool1.getOrigin().getY();
    tf_H_Cam_Tool1.transform.translation.z = H_Cam_Tool1.getOrigin().getZ();
    tf_H_Cam_Tool1.transform.rotation.w = H_Cam_Tool1.getRotation().w();
    tf_H_Cam_Tool1.transform.rotation.x = H_Cam_Tool1.getRotation().x();
    tf_H_Cam_Tool1.transform.rotation.y = H_Cam_Tool1.getRotation().y();
    tf_H_Cam_Tool1.transform.rotation.z = H_Cam_Tool1.getRotation().z();

    // using data retrieved from subscriber
    tf::Pose H_Tool1_Base1;
    tf::poseMsgToTF(rec_msg_1, H_Tool1_Base1);

    // // for non-moving da Vinci arm holding camera
    // geometry_msgs::Pose temp3;
    // temp3.position.x = ;
    // temp3.position.y = ;
    // temp3.position.z = ;
    // temp3.orientation.w = ;
    // temp3.orientation.x = ;
    // temp3.orientation.y = ;
    // temp3.orientation.z = ;
    // tf::Pose H_Tool1_Base1;
    // tf::poseMsgToTF(temp3, H_Tool1_Base1);
  
    tf::Transform H_Base1_Tool1 = H_Tool1_Base1.inverse();


    // Tooltip 2 position (including base frame, ie., wrt camera. That's why subscribed topic is NOT local.)
    tf::Pose H_Tool2;
    tf::poseMsgToTF(rec_msg_2, H_Tool2);
    geometry_msgs::TransformStamped tf_H_Tool2;
  
    tf_H_Tool2.header.stamp = ros::Time::now();
    tf_H_Tool2.header.frame_id = "new_cam";
    tf_H_Tool2.child_frame_id = "new_tool2";
    tf_H_Tool2.transform.translation.x = H_Tool2.getOrigin().getX();
    tf_H_Tool2.transform.translation.y = H_Tool2.getOrigin().getY();
    tf_H_Tool2.transform.translation.z = H_Tool2.getOrigin().getZ();
    tf_H_Tool2.transform.rotation.w = H_Tool2.getRotation().w();
    tf_H_Tool2.transform.rotation.x = H_Tool2.getRotation().x();
    tf_H_Tool2.transform.rotation.y = H_Tool2.getRotation().y();
    tf_H_Tool2.transform.rotation.z = H_Tool2.getRotation().z();


    // Tooltip 3 position (including base frame, ie., wrt camera. That's why subscribed topic is NOT local.)
    tf::Pose H_Tool3;
    tf::poseMsgToTF(rec_msg_3, H_Tool3);
    geometry_msgs::TransformStamped tf_H_Tool3;
  
    tf_H_Tool3.header.stamp = ros::Time::now();
    tf_H_Tool3.header.frame_id = "new_cam";
    tf_H_Tool3.child_frame_id = "new_tool3";
    tf_H_Tool3.transform.translation.x = H_Tool3.getOrigin().getX();
    tf_H_Tool3.transform.translation.y = H_Tool3.getOrigin().getY();
    tf_H_Tool3.transform.translation.z = H_Tool3.getOrigin().getZ();
    tf_H_Tool3.transform.rotation.w = H_Tool3.getRotation().w();
    tf_H_Tool3.transform.rotation.x = H_Tool3.getRotation().x();
    tf_H_Tool3.transform.rotation.y = H_Tool3.getRotation().y();
    tf_H_Tool3.transform.rotation.z = H_Tool3.getRotation().z();


    // transformation chains
    tf::Pose H_Base2_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base2_Base1;
    tf::Pose H_Base3_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base3_Base1;

    // for PSM2
    geometry_msgs::Pose msg2;
    tf::poseTFToMsg(H_Base2_Cam, msg2);

    // for PSM3
    geometry_msgs::Pose msg3;
    tf::poseTFToMsg(H_Base3_Cam, msg3);

    // tf
    tf::Transform H_Cam_Base2 = H_Base2_Cam.inverse(); 
    geometry_msgs::TransformStamped tf_H_Cam_Base2;
  
    tf_H_Cam_Base2.header.stamp = ros::Time::now();
    tf_H_Cam_Base2.header.frame_id = "base2";
    tf_H_Cam_Base2.child_frame_id = "new_cam_2";
    tf_H_Cam_Base2.transform.translation.x = H_Cam_Base2.getOrigin().getX();
    tf_H_Cam_Base2.transform.translation.y = H_Cam_Base2.getOrigin().getY();
    tf_H_Cam_Base2.transform.translation.z = H_Cam_Base2.getOrigin().getZ();
    tf_H_Cam_Base2.transform.rotation.w = H_Cam_Base2.getRotation().w();
    tf_H_Cam_Base2.transform.rotation.x = H_Cam_Base2.getRotation().x();
    tf_H_Cam_Base2.transform.rotation.y = H_Cam_Base2.getRotation().y();
    tf_H_Cam_Base2.transform.rotation.z = H_Cam_Base2.getRotation().z();

    // tf
    tf::Transform H_Cam_Base3 = H_Base3_Cam.inverse(); 
    geometry_msgs::TransformStamped tf_H_Cam_Base3;
  
    tf_H_Cam_Base3.header.stamp = ros::Time::now();
    tf_H_Cam_Base3.header.frame_id = "base3";
    tf_H_Cam_Base3.child_frame_id = "new_cam_3";
    tf_H_Cam_Base3.transform.translation.x = H_Cam_Base3.getOrigin().getX();
    tf_H_Cam_Base3.transform.translation.y = H_Cam_Base3.getOrigin().getY();
    tf_H_Cam_Base3.transform.translation.z = H_Cam_Base3.getOrigin().getZ();
    tf_H_Cam_Base3.transform.rotation.w = H_Cam_Base3.getRotation().w();
    tf_H_Cam_Base3.transform.rotation.x = H_Cam_Base3.getRotation().x();
    tf_H_Cam_Base3.transform.rotation.y = H_Cam_Base3.getRotation().y();
    tf_H_Cam_Base3.transform.rotation.z = H_Cam_Base3.getRotation().z();


    // setting base frames
    psm2_pub.publish(msg2);
    psm3_pub.publish(msg3);
    // ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

    if (baseFrameCount%10==0)
    	ROS_INFO("Updating base frames of PSM 2 and PSM3...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
    	baseFrameCount = 0;

    // sending transforms to visualize on rviz
    br.sendTransform(tf_H_Cam_Tool1);
    br.sendTransform(tf_H_Cam_Base2);
    br.sendTransform(tf_H_Cam_Base3);
    br.sendTransform(tf_H_Tool2);
    br.sendTransform(tf_H_Tool3);

    // setting registration rotation of MTML-PSM2
    geometry_msgs::Quaternion q2;
    q2.w = 0;
    q2.x = 1;
    q2.y = 0;
    q2.z = 0;
    
    psm2_rot_pub.publish(q);

    // setting registration rotation of MTMR-PSM3
    geometry_msgs::Quaternion q3;
    q3.w = 0;
    q3.x = 1;
    q3.y = 0;
    q3.z = 0;
    
    psm2_rot_pub.publish(q2);
    psm3_rot_pub.publish(q3);

    if(regRotFlag==0)
    {
    	ROS_INFO("Setting registration rotation for both teleop pairs");
    	regRotFlag = 1;
    }


    // TELEOPERATION LOGIC ------------
    // During teleoperation:
    // * Head sensor is required to enable teleoperation of both teleop pairs MTML-PSM2 and MTMR-PSM3
    //		- Callback to subscribe to topic /dvrk/footpedals/operatorpresent.
    //		- The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates head sensed (1) or not (0).
    
    /*

    std_msgs::String MTML_PSM2_State, MTMR_PSM3_State, MTML_PSM3_State, MTMR_PSM2_State;
    std::stringstream s1, s2, ss;

    // checking for presence of operator (head)
    if (headPressed == 1)
    {
    	// checking if CAMERA footpedal has been pressed
    	if (camPressed == 1)
    	{
    		// enabling MTMx-PSM2 teleop // enabling pickup camera arm teleop
    		s1 << "ENABLED";
    		MTML_PSM2_State.data = s1.str();
    		MTMR_PSM2_State.data = s1.str();

    		// freezing MTMx-PSM1 teleop // freezing non-pickup camera arm teleop
    		s2 << "ALIGNING_MTM";
  	    	MTMR_PSM3_State.data = s2.str();
  	    	MTML_PSM3_State.data = s2.str();
    	}
    	else
    	{
    		// freezing MTMx-PSM2 teleop // freezing pickup camera arm teleop
    		s1 << "ALIGNING_MTM";
    		MTML_PSM2_State.data = s1.str();
    		MTMR_PSM2_State.data = s1.str();

    		// enabling MTMx-PSM1 teleop // enabling non-pickup camera arm teleop
    		s2 << "ENABLED";
  	    	MTMR_PSM3_State.data = s2.str();
  	    	MTML_PSM3_State.data = s2.str();
    	}
    }
    else
    {
    	// freezing all teleops
    	ss << "ALIGNING_MTM";
    	MTMR_PSM3_State.data = ss.str();
    	MTML_PSM3_State.data = ss.str();
    	MTMR_PSM2_State.data = ss.str();
    	MTML_PSM2_State.data = ss.str();
    }

    // publishing to only active teleop pairs
    if (switchCount%2==1)
    {
    	MTMR_PSM3_pub.publish(MTMR_PSM3_State);
    	MTML_PSM2_pub.publish(MTML_PSM2_State);	
    }
    else
    {
    	MTMR_PSM2_pub.publish(MTMR_PSM2_State);
    	MTML_PSM3_pub.publish(MTML_PSM3_State);	
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
	*/

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}