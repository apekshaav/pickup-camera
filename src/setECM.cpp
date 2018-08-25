/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of the two da Vinci arms PSM2 (Green Arm) and PSM3 (Red Arm) with respect to the endoscope (ECM tip coordinate frame).
 */

#include <vector>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/Joy.h>

// TF
#include <tf2/LinearMath/Quaternion.h>

std::vector<int> isHead;
int headPressed;

void callbackHead(const sensor_msgs::Joy::ConstPtr& msg)
{
  isHead = msg->buttons;
  headPressed = isHead[0];

  ROS_INFO("Head pressed: %d", headPressed);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "setECM");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber head_sub = n.subscribe("/dvrk/footpedals/operatorpresent", 1000, callbackHead);
  
  // publishers
  ros::Publisher psm2_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  ros::Publisher psm3_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM3/set_base_frame", 1000);
  
  ros::Publisher psm2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  ros::Publisher psm3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM3/set_registration_rotation", 1000);

  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  ros::Publisher MTMR_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM1/set_desired_state", 1000);

  // ros::Publisher switch_pub = n.advertise<std_msgs::String>("/switch", 1000);
  ros::Publisher teleop_switch_pub = n.advertise<diagnostic_msgs::KeyValue>("/dvrk/console/teleop/select_teleop_psm", 1000);
  
  
  ros::Rate loop_rate(10);
  
  int switchFlag = 0;
  int baseFrameCount = 0;
  int regRotFlag = 0;
  while (ros::ok())
  {
    

  	//setting initial teleop pairs to MTML-PSM2 and MTMR-PSM3
	diagnostic_msgs::KeyValue teleopPair;
    if (switchFlag == 0)
    {	

      ros::Duration(1.5).sleep();
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

      switchFlag = 1;
    }
    

    // transform from PSM2 Base Frame to ECM tip frame
    geometry_msgs::Pose msg2;
    msg2.position.x = 0.2282;
    msg2.position.y = 0.2876;
    msg2.position.z = 1.4630;
    msg2.orientation.w = 0.6442;
    msg2.orientation.x = 0.5436;
    msg2.orientation.y = -0.3745;
    msg2.orientation.z = 0.3862;


    // transform from PSM3 Base Frame to ECM tip frame
    geometry_msgs::Pose msg3;
    msg3.position.x = 0.0037;
    msg3.position.y = -0.0169;
    msg3.position.z = 1.5176;
    msg3.orientation.w = 0.5814;
    msg3.orientation.x = 0.5487;
    msg3.orientation.y = 0.3935;
    msg3.orientation.z = -0.4539;

    if (baseFrameCount%10==0)
      ROS_INFO("Updating base frame of PSM 2 and PSM3...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
        baseFrameCount = 0;

    psm2_pub.publish(msg2);
    psm3_pub.publish(msg3);

    // setting registration rotation of MTML-PSM2
    geometry_msgs::Quaternion q2;
    q2.w = 0;
    q2.x = 0;
    q2.y = -0.7071; 
    q2.z = 0.7071;

    psm2_rot_pub.publish(q2);

    // setting registration rotation of MTMR-PSM3
    geometry_msgs::Quaternion q3;
    q3.w = 0;
    q3.x = 0;
    q3.y = -0.7071; 
    q3.z = 0.7071;

    psm3_rot_pub.publish(q3);

    if(regRotFlag==0)
    {
    	ROS_INFO("Setting registration rotation for both teleop pairs");
      regRotFlag = 1;
    }

    // TELEOPERATION LOGIC ------
    // During teleoperation:
    // * Head sensor is required to enable the teleoperation of MTMR-PSM1 (camera arm)
    //    - Callback to subscribe to topic /dvrk/footpedals/operatorpresent.
    //    - The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates head sensed (1) or not (0).
    // * MTML-PSM2 (non-camera arm) remains frozen all the time, even when head is sensed.

    /*std_msgs::String MTML_PSM2_State, MTMR_PSM1_State;
    std::stringstream s1, s2, ss;

    // checking for presence of operator (head)
    if (headPressed == 1)
    {
      // freezing MTML-PSM2 teleop // freezing pickup camera arm teleop
        s1 << "ALIGNING_MTM";
        MTML_PSM2_State.data = s1.str();

        // enabling MTMR-PSM1 teleop // enabling non-pickup camera arm teleop
        s2 << "ENABLED";
          MTMR_PSM1_State.data = s2.str();
    }
    else
    {
      // freezing all teleops
      ss << "ALIGNING_MTM";
      MTMR_PSM1_State.data = ss.str();
      MTML_PSM2_State.data = ss.str();
    }

    MTMR_PSM1_pub.publish(MTMR_PSM1_State);
    MTML_PSM2_pub.publish(MTML_PSM2_State); 
    */
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}