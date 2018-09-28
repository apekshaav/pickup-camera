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

std::vector<int> isCam;
int camPressed, switchPressed;

// Callback functions
void callbackCam(const sensor_msgs::Joy::ConstPtr& msg)
{
  isCam = msg->buttons;
  camPressed = isCam[0];

  ROS_INFO("Cam pressed: %d", camPressed);
}

void callbackSwitch(const std_msgs::String::ConstPtr& msg)
{
  switchPressed = 1;
  ROS_INFO("Switching enabled: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "Control");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber cam_sub = n.subscribe("/dvrk/footpedals/camera", 1000, callbackCam);
  ros::Subscriber switch_sub = n.subscribe("/pickup/switch", 1000, callbackSwitch);
  
  // publishers
  ros::Publisher psm2_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  ros::Publisher psm3_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM3/set_base_frame", 1000);
  
  ros::Publisher MTML_PSM2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  ros::Publisher MTMR_PSM3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM3/set_registration_rotation", 1000);
  ros::Publisher MTMR_PSM2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM2/set_registration_rotation", 1000);
  ros::Publisher MTML_PSM3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM3/set_registration_rotation", 1000);

  ros::Publisher switch_pub = n.advertise<std_msgs::String>("/pickup/switch", 1000);
  ros::Publisher teleop_pub = n.advertise<diagnostic_msgs::KeyValue>("/dvrk/console/teleop/select_teleop_psm", 1000);
  
  
  ros::Rate loop_rate(100);
  
  // initialize

  // transform from PSM2 Base Frame to ECM tip frame
  geometry_msgs::Pose H_Base2_ECM;
  H_Base2_ECM.position.x = 0.2282;
  H_Base2_ECM.position.y = 0.2876;
  H_Base2_ECM.position.z = 1.4630;
  H_Base2_ECM.orientation.w = 0.6442;
  H_Base2_ECM.orientation.x = 0.5436;
  H_Base2_ECM.orientation.y = -0.3745;
  H_Base2_ECM.orientation.z = 0.3862;

  // transform from PSM3 Base Frame to ECM tip frame
  geometry_msgs::Pose H_Base3_ECM;
  H_Base3_ECM.position.x = 0.0037;
  H_Base3_ECM.position.y = -0.0169;
  H_Base3_ECM.position.z = 1.5176;
  H_Base3_ECM.orientation.w = 0.5814;
  H_Base3_ECM.orientation.x = 0.5487;
  H_Base3_ECM.orientation.y = 0.3935;
  H_Base3_ECM.orientation.z = -0.4539;  

  // setting registration rotation of MTML-PSM2
  geometry_msgs::Quaternion q2;
  q2.w = 0;
  q2.x = 0;
  q2.y = -0.7071; 
  q2.z = 0.7071;

  // setting registration rotation of MTMR-PSM3
  geometry_msgs::Quaternion q3;
  q3.w = 0;
  q3.x = 0;
  q3.y = -0.7071; 
  q3.z = 0.7071;

  int switchFlag = 0;
  int switchCount = 0;
  int baseFrameCount = 0;
  int regRotFlag = 0;
  while (ros::ok())
  {

    psm2_pub.publish(H_Base2_ECM);
    psm3_pub.publish(H_Base3_ECM);

    if (baseFrameCount%100==0)
      ROS_INFO("Control: ECM: Updating base frames...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
        baseFrameCount = 0;


    MTML_PSM2_rot_pub.publish(q2);
    MTMR_PSM2_rot_pub.publish(q2);

    MTMR_PSM3_rot_pub.publish(q3);
    MTML_PSM3_rot_pub.publish(q3);

    if(regRotFlag==0)
    {
    	ROS_INFO("Control: ECM: Setting registration rotation for both teleop pairs");
      regRotFlag = 1;
    }


    std_msgs::String switchString;
    std::stringstream stemp;
    // switching teleop pairs first:
    if (switchFlag==0)
    {
        stemp << "yes";
        switchString.data = stemp.str();
        switch_pub.publish(switchString);
        switchFlag = 1;
        ros::Duration(1).sleep();
        // switchCount++;
    }

    // SWITCHING LOGIC
    diagnostic_msgs::KeyValue teleopPair;
    // switching teleop pairs when message is published to topic /switch
    if (switchPressed == 1)
    { 
      switchPressed = 0;
      if (switchCount%2==1)
      {
        teleopPair.key = "MTML";
        teleopPair.value = "";
        teleop_pub.publish(teleopPair);
        ROS_INFO("Switching has begun...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTMR";
        teleopPair.value = "PSM2";
        teleop_pub.publish(teleopPair);
        // ROS_INFO("...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTML";
        teleopPair.value = "PSM3";
        teleop_pub.publish(teleopPair);
        ROS_INFO("Switching has finished.");
        ros::Duration(1).sleep(); // sleep for a second  
      }
      else
      {
        teleopPair.key = "MTML";
        teleopPair.value = "";
        teleop_pub.publish(teleopPair);
        ROS_INFO("Switching has begun...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTMR";
        teleopPair.value = "PSM3";
        teleop_pub.publish(teleopPair);
        // ROS_INFO("...");
        ros::Duration(1).sleep(); // sleep for a second

        teleopPair.key = "MTML";
        teleopPair.value = "PSM2";
        teleop_pub.publish(teleopPair);
        ROS_INFO("Switching has finished.");
        ros::Duration(1).sleep(); // sleep for a second  
      }
      switchCount++;
      // ros::Duration(0.25).sleep();
    }


    // if CAM is pressed, arms must be switched
    if (camPressed)
    {
      camPressed = 0;
      // switchCount++;
      std_msgs::String switchString2;
      std::stringstream stemp2;
      stemp2 << "yes";
      switchString2.data = stemp2.str();
      switch_pub.publish(switchString2);
      ros::Duration(1).sleep();
    }
    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}