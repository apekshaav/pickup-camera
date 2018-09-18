/**
 * This script is used to set the scale for all teleoperation pairs uniformly.
 */

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 newScale;
bool msgReceived = false;

void callback(const std_msgs::Float32::ConstPtr& msg)
{
  newScale.data = msg->data;
  msgReceived = true;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "setECM");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber scale_sub = n.subscribe("/set_teleop_scale", 1000, callback);
  
  // publishers
  ros::Publisher scale_pub = n.advertise<std_msgs::Float32>("/set_teleop_scale", 1000);

  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::Float32>("/dvrk/MTML_PSM2/set_scale", 1000);
  ros::Publisher MTMR_PSM3_pub = n.advertise<std_msgs::Float32>("/dvrk/MTMR_PSM3/set_scale", 1000);
  ros::Publisher MTMR_PSM2_pub = n.advertise<std_msgs::Float32>("/dvrk/MTMR_PSM2/set_scale", 1000);
  ros::Publisher MTML_PSM3_pub = n.advertise<std_msgs::Float32>("/dvrk/MTML_PSM3/set_scale", 1000);  
  
  ros::Rate loop_rate(10);
  
  std_msgs::Float32 scale;

  while (ros::ok())
  {
    
    //setting scale

    if(!msgReceived)
      scale.data = 0.2; // default
    else
      scale.data = newScale.data;

    MTML_PSM2_pub.publish(scale);
    MTMR_PSM3_pub.publish(scale);
    MTMR_PSM2_pub.publish(scale);
    MTML_PSM3_pub.publish(scale);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}