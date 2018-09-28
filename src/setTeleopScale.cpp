/**
 * This script is used to set the scale for all teleoperation pairs uniformly.
 */

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 scale;
bool scale_set = false;

void callback(const std_msgs::Float32::ConstPtr& msg)
{
  scale.data = msg->data;
  scale_set = true;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "TeleopScale");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber scale_sub = n.subscribe("/pickup/set_teleop_scale", 1000, callback);
  
  // publishers
  // ros::Publisher scale_pub = n.advertise<std_msgs::Float32>("/pickup/et_teleop_scale", 1000);

  ros::Publisher MTML_PSM2_pub = n.advertise<std_msgs::Float32>("/dvrk/MTML_PSM2/set_scale", 1000);
  ros::Publisher MTMR_PSM3_pub = n.advertise<std_msgs::Float32>("/dvrk/MTMR_PSM3/set_scale", 1000);
  ros::Publisher MTMR_PSM2_pub = n.advertise<std_msgs::Float32>("/dvrk/MTMR_PSM2/set_scale", 1000);
  ros::Publisher MTML_PSM3_pub = n.advertise<std_msgs::Float32>("/dvrk/MTML_PSM3/set_scale", 1000);  
  
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    
    // polling callback
    ros::spinOnce();

    //setting scale
    if(scale_set)
    {
		  scale_set = false;
		  MTML_PSM2_pub.publish(scale);
    	MTMR_PSM3_pub.publish(scale);
    	MTMR_PSM2_pub.publish(scale);
		  MTML_PSM3_pub.publish(scale);

		  ROS_INFO("Teleop scale set to: %f", scale.data);
    }

    loop_rate.sleep();
  }


  return 0;
}
