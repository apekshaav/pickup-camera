/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of a da Vinci PSM with respect to the endoscope.
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>

std::vector<int> isCam, isCoag;
int camPressed, coagPressed;

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



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ecm_pub");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber cam_sub = n.subscribe("/dvrk/footpedals/camera", 1000, callbackCam);
  ros::Subscriber coag_sub = n.subscribe("/dvrk/footpedals/coag", 1000, callbackCoag);
  
  // publishers
  ros::Publisher ECM_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  // ros::Publisher rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::String>("/dvrk/MTML_PSM2/set_desired_state", 1000);
  ros::Publisher MTMR_PSM1_pub = n.advertise<std_msgs::String>("/dvrk/MTMR_PSM1/set_desired_state", 1000);
  
  ros::Rate loop_rate(10);
  
  int count = 0;

  while (ros::ok())
  {
    
    // transform from (my) world frame to ECM frame
    geometry_msgs::Pose msg;
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 0;
    msg.orientation.w = 1;
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;

    ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    // ROS_INFO("Updating base frame of PSM 2...");

    ECM_pub.publish(msg);

    // // setting registration rotation of MTML-PSM2
    // geometry_msgs::Quaternion q;
    // q.w = 0;
    // q.x = 0;
    // q.y = 1;
    // q.z = 0;

    // if(count==0)
    // {
    // 	ROS_INFO("Setting registration rotation for MTML-PSM2");
    // }

    // rot_pub.publish(q);
	// count = count + 1;


    // During teleoperation:
    // 
    // * Use 'camera' footpedal to enable teleoperation between MTMR-PSM1 (camera arm).
    //      - Teleoperation between MTML-PSM2 will be frozen during this time. 
    //      - Callback to subscribe to topic /dvrk/footpedals/camera.
    //      - The messages received are the form sensor_msgs/Joy. The 'buttons' value indicates pressed footpedal (1) or not (0).
    //      - When camPressed = 1 (pressed), enable MTML-PSM2 teleop by publishing to topic /dvrk
    // * The usual COAG footpedal enables teleoperation between MTML-PSM2 (non-camera arm).
    //      - Teleoperation between MTMR-PSM1 will be frozen during this time.


    std_msgs::String MTML_PSM2_State, MTMR_PSM1_State;
    std::stringstream s1, s2;

    if (camPressed == 1)
    {   
        s1 << "ENABLED";
        MTML_PSM2_State.data = s1.str();
        // ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());
    }
    else
    {
        s1 << "ALIGNING_MTM";
        MTML_PSM2_State.data = s1.str();
        // ROS_INFO("MTML_PSM2_State: %s", MTML_PSM2_State.data.c_str());
    }

    MTML_PSM2_pub.publish(MTML_PSM2_State);


    // coag footpedal - right - MTMR-PSM1 teleop
    if (coagPressed == 1)
    {
        s2 << "ENABLED";
        MTMR_PSM1_State.data = s2.str();
        // ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());
    }
    else
    {
        s2 << "ALIGNING_MTM";
        MTMR_PSM1_State.data = s2.str();
        // ROS_INFO("MTMR_PSM1_State: %s", MTMR_PSM1_State.data.c_str());
    }

    MTMR_PSM1_pub.publish(MTMR_PSM1_State);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}