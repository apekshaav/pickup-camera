/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of a da Vinci PSM with respect to the endoscope.
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ecm_pub");
  ros::NodeHandle n;

  // subscribers
  
  // publishers
  ros::Publisher ECM_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  
  ros::Rate loop_rate(10);
  
  int baseFrameCount = 0;

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

    if (baseFrameCount%10==0)
        ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", msg.position.x, msg.position.y, msg.position.z, msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    // ROS_INFO("Updating base frame of PSM 2...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
        baseFrameCount = 0;

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


    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}