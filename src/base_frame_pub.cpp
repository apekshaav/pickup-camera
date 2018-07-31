/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * for a da Vinci PSM with respect to a pick-up camera held by another PSM.
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sstream>

geometry_msgs::Pose rec_msg_1, rec_msg_2;
std::vector<int> isCam, isCoag, isHead;
int camPressed, coagPressed, headPressed;

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

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "base_frame_pub");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber sub1 = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, callbackPSM1);
  ros::Subscriber sub2 = n.subscribe("/dvrk/PSM2/position_cartesian_current", 1000, callbackPSM2);
  ros::Subscriber cam_sub = n.subscribe("/dvrk/footpedals/camera", 1000, callbackCam);
  ros::Subscriber coag_sub = n.subscribe("/dvrk/footpedals/coag", 1000, callbackCoag);
  ros::Subscriber head_sub = n.subscribe("/dvrk/footpedals/operatorpresent", 1000, callbackHead);
  
  // publishers
  ros::Publisher transform_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  ros::Publisher rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  ros::Publisher MTMR_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM1/set_desired_state", 1000);
  ros::Publisher MTML_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM1/set_desired_state", 1000);
  ros::Publisher MTMR_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM2/set_desired_state", 1000);
  // ros::Publisher switch_pub = n.advertise<diagnostic_msgs::KeyValue>("/dvrk/console/teleop/select_teleop_psm", 1000);

  ros::Rate loop_rate(10);

  int ct = 1; 
  int count = 0;
  while (ros::ok())
  {
    
    //////////////////////////////////////////////////////////////////////////////////
    // Transforms                                                                   //
    // ----------                                                                   //
    // H_Base1_W      : Base 1 to World                                             //
    // H_Base2_W      : Base 2 to World                                             //
    // H_Base2_Base1  : Base 2 to Base 1                                            //
    // H_Tool1_Base1  : Tooltip 1 to Base 1                                         //
    // H_Tool2_Base2  : Tooltip 2 to Base 2                                         //
    // H_Base1_Tool1  : Base 1 to Tooltip 1                                         //
    // H_Tool1_Cam    : Tooltip 1 to Pick-Up Camera                                 //
    // H_Cam_Tool1    : Pick-Up Camera to Tooltip 1                                 //
    // H_Cam          : Final Transform - Base 2 to Pick-Up Camera                  //
    //////////////////////////////////////////////////////////////////////////////////

    // creating object to broadcast transforms 
    static tf2_ros::TransformBroadcaster br;

    // for H_Base2_Base1
    geometry_msgs::Pose temp;
    temp.position.x = 0.0155;
    temp.position.y = 0.1194;
    temp.position.z =  -0.0751;
    temp.orientation.w = 0.6210;
    temp.orientation.x = 0.3224;
    temp.orientation.y = 0.3590;
    temp.orientation.z = 0.6176;
    
    tf::Pose H_Base2_Base1;
    tf::poseMsgToTF(temp, H_Base2_Base1);     
    
    // // for H_Tool1_Cam (from hand-eye calibration)
    // geometry_msgs::Pose temp1;
    // temp1.position.x = -0.1198;
    // temp1.position.y = -0.0782;
    // temp1.position.z = -0.1716;
    // temp1.orientation.w = 0.7110;
    // temp1.orientation.x = 0.2490;
    // temp1.orientation.y = -0.6566;
    // temp1.orientation.z = 0.0374;

    // from solidworks model:
    // for H_Tool1_Cam
    geometry_msgs::Pose temp1;
    temp1.position.x = 0.0338;
    temp1.position.y = 0.0;
    temp1.position.z = 0.0;
    temp1.orientation.w = 1; //0.7071;
    temp1.orientation.x = 0; //0.7071;
    temp1.orientation.y = 0.0;
    temp1.orientation.z = 0.0;
    
    tf::Pose H_Tool1_Cam;
    tf::poseMsgToTF(temp1, H_Tool1_Cam);

    // ROS_INFO("1: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Tool1_Cam.getOrigin().getX(), H_Tool1_Cam.getOrigin().getY(), H_Tool1_Cam.getOrigin().getZ(), H_Tool1_Cam.getRotation().w(), H_Tool1_Cam.getRotation().x(), H_Tool1_Cam.getRotation().y(), H_Tool1_Cam.getRotation().z());

    tf::Transform H_Cam_Tool1 = H_Tool1_Cam.inverse(); //(H_Tool1_Cam);
    // H_Cam_Tool1.inverse();
    // ROS_INFO("2: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Cam_Tool1.getOrigin().getX(), H_Cam_Tool1.getOrigin().getY(), H_Cam_Tool1.getOrigin().getZ(), H_Cam_Tool1.getRotation().w(), H_Cam_Tool1.getRotation().x(), H_Cam_Tool1.getRotation().y(), H_Cam_Tool1.getRotation().z());

    // broadcasting
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

    // ROS_INFO("1: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Tool1_Base1.getOrigin().getX(), H_Tool1_Base1.getOrigin().getY(), H_Tool1_Base1.getOrigin().getZ(), H_Tool1_Base1.getRotation().w(), H_Tool1_Base1.getRotation().x(), H_Tool1_Base1.getRotation().y(), H_Tool1_Base1.getRotation().z());

    // // for non-moving da Vinci arm holding camera
    // geometry_msgs::Pose temp2;
    // temp2.position.x = -0.0279;
    // temp2.position.y = -0.0036;
    // temp2.position.z = -0.0762;
    // temp2.orientation.w = -0.2461;
    // temp2.orientation.x = 0.4265;
    // temp2.orientation.y = 0.8089;
    // temp2.orientation.z = -0.3210;
    // tf::Pose H_Tool1_Base1;
    // tf::poseMsgToTF(temp2, H_Tool1_Base1);

  
    tf::Transform H_Base1_Tool1 = H_Tool1_Base1.inverse(); //(H_Tool1_Base1);
    // H_Base1_Tool1.inverse();

    // ROS_INFO("2: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Base1_Tool1.getOrigin().getX(), H_Base1_Tool1.getOrigin().getY(), H_Base1_Tool1.getOrigin().getZ(), H_Base1_Tool1.getRotation().w(), H_Base1_Tool1.getRotation().x(), H_Base1_Tool1.getRotation().y(), H_Base1_Tool1.getRotation().z());
    

    // tf::Transform T1;
    // T1.mult(H_Base1_Tool1, H_Base2_Base1);

    // tf::Transform H_Cam;
    // H_Cam.mult(H_Tool1_Cam, T1);


    // Tooltip 2 position (including base frame, ie., wrt camera)
    tf::Pose H_Tool2;
    tf::poseMsgToTF(rec_msg_2, H_Tool2);
	// broadcasting
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


    tf::Pose H_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base2_Base1;
    // ROS_INFO("1: Position: [%f %f %f] Orientation: [%f %f %f %f]", H_Final.getOrigin().getX(), H_Final.getOrigin().getY(), H_Final.getOrigin().getZ(), H_Final.getRotation().w(), H_Final.getRotation().x(), H_Final.getRotation().y(), H_Final.getRotation().z());

    geometry_msgs::Pose msg;
    tf::poseTFToMsg(H_Cam, msg);

    // broadcasting
    tf::Transform H_Cam_Base2 = H_Cam.inverse(); //(H_Cam);
    // H_Cam_Base2.inverse();
    geometry_msgs::TransformStamped tf_H_Cam_Base2;
  
    tf_H_Cam_Base2.header.stamp = ros::Time::now();
    tf_H_Cam_Base2.header.frame_id = "base2";
    tf_H_Cam_Base2.child_frame_id = "new_cam";
    tf_H_Cam_Base2.transform.translation.x = H_Cam_Base2.getOrigin().getX();
    tf_H_Cam_Base2.transform.translation.y = H_Cam_Base2.getOrigin().getY();
    tf_H_Cam_Base2.transform.translation.z = H_Cam_Base2.getOrigin().getZ();
    tf_H_Cam_Base2.transform.rotation.w = H_Cam_Base2.getRotation().w();
    tf_H_Cam_Base2.transform.rotation.x = H_Cam_Base2.getRotation().x();
    tf_H_Cam_Base2.transform.rotation.y = H_Cam_Base2.getRotation().y();
    tf_H_Cam_Base2.transform.rotation.z = H_Cam_Base2.getRotation().z();


    // ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    //ROS_INFO("Updating base frame of PSM 2...");

    //transform_pub.publish(msg);

    // sending transforms to visualize on rviz
    br.sendTransform(tf_H_Cam_Tool1);
    br.sendTransform(tf_H_Cam_Base2);
    br.sendTransform(tf_H_Tool2);

    // setting registration rotation of MTML-PSM2
    geometry_msgs::Quaternion q;
    q.w = 0;
    q.x = 0;
    q.y = 1;
    q.z = 0;

    if(count==0)
    {
    	ROS_INFO("Setting registration rotation for MTML-PSM2");
    }

    //rot_pub.publish(q);



    // During teleoperation:
    // * Head sensor is required to enable teleoperation of both teleop pairs MTMR-PSM1 (camera arm) and MTML-PSM2 (tool arm)
    //		- Callback to subscribe to topic /dvrk/footpedals/operatorpresent.
    //		- The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates head sensed (1) or not (0).
    // * CAMERA footpedal determines which teleop pair is ACTIVELY enabled at the time
    //		- The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates footpedal pressed (1) or not (0).
    //		- When CAMERA footpedal is NOT pressed:
    // 			-- When headPressed = 1 (head sensed) AND camPressed = 0 (not pressed), enable MTML-PSM2 (tool/non-camera arm) teleop by publishing to topic /dvrk
    //			-- Teleoperation between MTMR-PSM1 will be frozen during this time.
    //		- When CAMERA footpedal is pressed:
    // 			-- When headPressed = 1 (head sensed) AND camPressed = 1 (pressed), enable MTMR-PSM1 teleop by publishing to topic /dvrk
    //			-- Teleoperation between MTML-PSM2 will be frozen during this time.
    
	// Switching PSMs
	// * COAG footpedal is used to make the switch    
   	// * The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates footpedal pressed (1) or not (0).
   	// * Each time COAG is pressed, the teleop pairs must be switched. Use a count variable. 
   	// * For all even counts, the original teleop pairs are used.
   	// * For all odd counts, the reverse teleop pairs are used.
    // * These counts must be integrated into the teleoperation code too.
    
    
    std_msgs::String MTML_PSM2_State, MTMR_PSM1_State, MTML_PSM1_State, MTMR_PSM2_State;
    std::stringstream s1, s2;

    // checking for presence of operator (head)
    if (headPressed == 1)
    {
    	// checking if CAMERA footpedal has been pressed
    	if (camPressed == 1)
    	{
    		// enabling MTMR-PSM1 teleop
    		s1 << "ENABLED";
    		MTMR_PSM1_State.data = s1.str();
    		// ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());

    		// freezing MTML-PSM2 teleop
    		s2 << "ALIGNING_MTM";
  	    	MTML_PSM2_State.data = s2.str();
    		// ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());	
    	}
    	else
    	{
    		// freezing MTMR-PSM1 teleop
    		s1 << "ALIGNING_MTM";
    		MTMR_PSM1_State.data = s1.str();
    		// ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());

    		// enabling MTML-PSM2 teleop
    		s2 << "ENABLED";
  	    	MTML_PSM2_State.data = s2.str();
    		// ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());		
    	}
    }
    else
    {
    	// freezing MTMR-PSM1 teleop
    	s1 << "ALIGNING_MTM";
    	MTMR_PSM1_State.data = s1.str();
    	// ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());

    	// freezing MTML-PSM2 teleop
    	s2 << "ALIGNING_MTM";
  	    MTML_PSM2_State.data = s2.str();
    	// ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());
    }

    MTMR_PSM1_pub.publish(MTMR_PSM1_State);
    MTML_PSM2_pub.publish(MTML_PSM2_State);

    


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


    // diagnostic_msgs::KeyValue teleopPair;
    // // switching teleop pairs when camera footpedal is pressed
    // if (camPressed == 1)
    // {	
    // 	if (ct%2==1)
    //   {
    //     teleopPair.key = "MTML";
    //     teleopPair.value = "";
    //     switch_pub.publish(teleopPair);
    //     ROS_INFO("Switching has begun...");
    //     ros::Duration(0.25).sleep(); // sleep for quarter a second

    //     teleopPair.key = "MTMR";
    //     teleopPair.value = "PSM2";
    //     switch_pub.publish(teleopPair);
    //     // ROS_INFO("...");
    //     ros::Duration(0.25).sleep(); // sleep for quarter a second

    //     teleopPair.key = "MTML";
    //     teleopPair.value = "PSM1";
    //     switch_pub.publish(teleopPair);
    //     ROS_INFO("Switching has finished.");
    //     ros::Duration(0.25).sleep(); // sleep for quater a second  
    //   }
    //   else
    //   {
    //     teleopPair.key = "MTML";
    //     teleopPair.value = "";
    //     switch_pub.publish(teleopPair);
    //     ROS_INFO("Switching has begun...");
    //     ros::Duration(0.25).sleep(); // sleep for quarter a second

    //     teleopPair.key = "MTMR";
    //     teleopPair.value = "PSM1";
    //     switch_pub.publish(teleopPair);
    //     // ROS_INFO("...");
    //     ros::Duration(0.25).sleep(); // sleep for quarter a second

    //     teleopPair.key = "MTML";
    //     teleopPair.value = "PSM2";
    //     switch_pub.publish(teleopPair);
    //     ROS_INFO("Switching has finished.");
    //     ros::Duration(0.25).sleep(); // sleep for quater a second  
    //   }
    //   ct++;
    // }



	count = count + 1;

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}