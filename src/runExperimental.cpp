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

// TODO: exit program when dvrk console is closed.


geometry_msgs::Pose rec_msg_1;
std::vector<int> isCam;
int camPressed, switchPressed;

void callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_msg_1 = msg->pose;

}

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
  
  ros::init(argc, argv, "Experimental");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber sub1 = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, callbackPSM1);

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


  // initialize

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

  // for H_Base2_Base1 - from MATLAB script
  geometry_msgs::Pose temp;
  temp.position.x = 0.0081;
  temp.position.y = 0.2767;
  temp.position.z = -0.0580;
  temp.orientation.w = 0.5645;
  temp.orientation.x = -0.0469;
  temp.orientation.y = -0.0250;
  temp.orientation.z = 0.8237;

  tf::Pose H_Base2_Base1;
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

  // setting registration rotations
  geometry_msgs::Quaternion q;
  q.w = 0;
  q.x = 1;
  q.y = 0;
  q.z = 0;

  // for ECM frame
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

  // setting registration rotation
  geometry_msgs::Quaternion q_ECM;
  q_ECM.w = 0;
  q_ECM.x = 0;
  q_ECM.y = -0.7071; 
  q_ECM.z = 0.7071;

  ros::Rate loop_rate(100);

  int switchFlag = 0;
  int camCount = 0;
  int switchCount = 1; 
  int baseFrameCount = 0;
  int regRotFlag = 0;
  while (ros::ok())
  {

    if (camPressed)
    {
        camPressed = 0;
        camCount++;
        switchFlag = 0;
    }
    
    // using data retrieved from subscriber
    tf::Pose H_Tool1_Base1;
    tf::poseMsgToTF(rec_msg_1, H_Tool1_Base1);
    tf::Transform H_Base1_Tool1 = H_Tool1_Base1.inverse();

    // transformation chains for pickup camera frame
    tf::Pose H_Base2_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base2_Base1;
    tf::Pose H_Base3_Cam = H_Tool1_Cam * H_Base1_Tool1 * H_Base3_Base1;

    // for PSM2
    geometry_msgs::Pose msg2;
    tf::poseTFToMsg(H_Base2_Cam, msg2);

    // for PSM3
    geometry_msgs::Pose msg3;
    tf::poseTFToMsg(H_Base3_Cam, msg3);

    // setting base frames
    if (camCount%2==0)
    {
        psm2_pub.publish(msg2);
        psm3_pub.publish(msg3);
        if (baseFrameCount%100==0)
            ROS_INFO("Experimental: Pickup: Updating base frames...");
        baseFrameCount++;
    }
    else
    {
        psm2_pub.publish(H_Base2_ECM);
        psm3_pub.publish(H_Base3_ECM);      
        if (baseFrameCount%100==0)
            ROS_INFO("Experimental: ECM: Updating base frames...");
        baseFrameCount++; 
    }
    // ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
    	baseFrameCount = 0;

    
    if (camCount%2==0)
    {
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
    }
    else
    {
        MTML_PSM2_rot_pub.publish(q_ECM);
        MTMR_PSM3_rot_pub.publish(q_ECM);
        MTMR_PSM2_rot_pub.publish(q_ECM);
        MTML_PSM3_rot_pub.publish(q_ECM);
    }
    

    if(regRotFlag==0)
    {
        ROS_INFO("Experimental: Setting registration rotation for both teleop pairs");
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
	

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}