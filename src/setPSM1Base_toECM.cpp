/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of the da Vinci PSM1 (Yellow Arm) with respect to the endoscope (ECM tip coordinate frame).
 */

#include <vector>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
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
  
  ros::init(argc, argv, "setPSM1Base_toECM");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber head_sub = n.subscribe("/dvrk/footpedals/operatorpresent", 1000, callbackHead);
  
  // publishers
  ros::Publisher ECM_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_base_frame", 1000);
  ros::Publisher rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM1/set_registration_rotation", 1000);
  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  ros::Publisher MTMR_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM1/set_desired_state", 1000);
  
  
  ros::Rate loop_rate(10);
  
  int baseFrameCount = 0;
  int regRotFlag = 0;
  while (ros::ok())
  {
    
    // transform from da Vinci world frame to ECM tip frame
    geometry_msgs::Pose msg;
    msg.position.x = -0.0305;
    msg.position.y = 0.0937;
    msg.position.z = 1.3383;
    msg.orientation.w =  0.6193;
    msg.orientation.x = 0.6410;
    msg.orientation.y = 0.3174;
    msg.orientation.z = -0.3239;

    if (baseFrameCount%10==0)
        // ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
      ROS_INFO("Updating base frame of PSM 1...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
        baseFrameCount = 0;

    ECM_pub.publish(msg);

    // setting registration rotation of MTMR-PSM1
    geometry_msgs::Quaternion q;
    q.w = 0;
    q.x = 1; //1;
    q.y = 0; //0;
    q.z = 0;

    rot_pub.publish(q);

    if(regRotFlag==0)
    {
    	ROS_INFO("Setting registration rotation for MTMR-PSM1");
      regRotFlag = 1;
    }

    // TELEOPERATION LOGIC ------
    // During teleoperation:
    // * Head sensor is required to enable the teleoperation of MTMR-PSM1 (camera arm)
    //    - Callback to subscribe to topic /dvrk/footpedals/operatorpresent.
    //    - The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates head sensed (1) or not (0).
    // * MTML-PSM2 (non-camera arm) remains frozen all the time, even when head is sensed.

    std_msgs::String MTML_PSM2_State, MTMR_PSM1_State;
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

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}