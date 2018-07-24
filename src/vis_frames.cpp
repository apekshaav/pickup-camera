/**
 * This script publishes tf transforms that can be visualized using rviz.
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sstream>

geometry_msgs::Pose rec_msg_1;
geometry_msgs::Pose rec_msg_2;

void callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_msg_1 = msg->pose;

  // ROS_INFO("Pose receieved");
}

void callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_msg_2 = msg->pose;

  // ROS_INFO("Pose receieved");
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "visualize");
  ros::NodeHandle n;

  ros::Subscriber sub_1 = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, callbackPSM1);
  ros::Subscriber sub_2 = n.subscribe("/dvrk/PSM2/position_cartesian_local_current", 1000, callbackPSM2);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    

    //////////////////////////////////////////////////////////////////////////////////
    // Transforms                                                                   //
    // ----------                                                                   //
    // H_Base1_W      : Base 1 to World                                             //
    // H_Base2_W      : Base 2 to World                                             //
    // H_Tool1_Base1  : Tooltip 1 to Base 1                                         //
    // H_Tool2_Base2  : Tooltip 2 to Base 2                                         //
    // H_Base1_Tool1  : Base 1 to Tooltip 1                                         //
    // H_Base2_Tool2  : Base 2 to Tooltip 2                                         //
    // H_Tool1_Cam    : Tooltip 1 to Pick-Up Camera                                 //
    // H_Cam          : Final Transform - Base 2 to Pick-Up Camera                  //
    //////////////////////////////////////////////////////////////////////////////////

    // creating object to broadcast transforms 
    static tf2_ros::TransformBroadcaster br;


    // for H_Base1_W (base frame of PSM1 wrt world)
    geometry_msgs::TransformStamped tf_H_Base1_W;
  
    tf_H_Base1_W.header.stamp = ros::Time::now();
    tf_H_Base1_W.header.frame_id = "world";
    tf_H_Base1_W.child_frame_id = "base1";
    tf_H_Base1_W.transform.translation.x = -0.0278;
    tf_H_Base1_W.transform.translation.y = -0.0425;
    tf_H_Base1_W.transform.translation.z = 0.1627;
    tf_H_Base1_W.transform.rotation.w = 0.9741;
    tf_H_Base1_W.transform.rotation.x = -0.0841;
    tf_H_Base1_W.transform.rotation.y = -0.1222;
    tf_H_Base1_W.transform.rotation.z = -0.1705;
    
  
    // for H_Base2_W (base frame of PSM2 wrt world)
    geometry_msgs::TransformStamped tf_H_Base2_W;
  
    tf_H_Base2_W.header.stamp = ros::Time::now();
    tf_H_Base2_W.header.frame_id = "world";
    tf_H_Base2_W.child_frame_id = "base2";
    tf_H_Base2_W.transform.translation.x = 0.1243;
    tf_H_Base2_W.transform.translation.y = 0.0050;
    tf_H_Base2_W.transform.translation.z = 0.1882;
    tf_H_Base2_W.transform.rotation.w = 0.9520;
    tf_H_Base2_W.transform.rotation.x = -0.0621;
    tf_H_Base2_W.transform.rotation.y = 0.0434;
    tf_H_Base2_W.transform.rotation.z = 0.2964;
  

    // using data retrieved from subscriber
    tf::Pose H_Tool1_Base1;
    tf::poseMsgToTF(rec_msg_1, H_Tool1_Base1);


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

    // broadcasting
    geometry_msgs::TransformStamped tf_H_Tool1_Base1;
  
    tf_H_Tool1_Base1.header.stamp = ros::Time::now();
    tf_H_Tool1_Base1.header.frame_id = "base1";
    tf_H_Tool1_Base1.child_frame_id = "tool1";
    tf_H_Tool1_Base1.transform.translation.x = H_Tool1_Base1.getOrigin().getX();
    tf_H_Tool1_Base1.transform.translation.y = H_Tool1_Base1.getOrigin().getY();
    tf_H_Tool1_Base1.transform.translation.z = H_Tool1_Base1.getOrigin().getZ();
    tf_H_Tool1_Base1.transform.rotation.w = H_Tool1_Base1.getRotation().w();
    tf_H_Tool1_Base1.transform.rotation.x = H_Tool1_Base1.getRotation().x();
    tf_H_Tool1_Base1.transform.rotation.y = H_Tool1_Base1.getRotation().y();
    tf_H_Tool1_Base1.transform.rotation.z = H_Tool1_Base1.getRotation().z();
  

    // using data retrieved from subscriber
    tf::Pose H_Tool2_Base2;
    tf::poseMsgToTF(rec_msg_2, H_Tool2_Base2);

    // PSM 3 tooltip wrt Base 3   
    geometry_msgs::TransformStamped tf_H_Tool2_Base2;
  
    tf_H_Tool2_Base2.header.stamp = ros::Time::now();
    tf_H_Tool2_Base2.header.frame_id = "base2";
    tf_H_Tool2_Base2.child_frame_id = "tool2";
    tf_H_Tool2_Base2.transform.translation.x = H_Tool2_Base2.getOrigin().getX();
    tf_H_Tool2_Base2.transform.translation.y = H_Tool2_Base2.getOrigin().getY();
    tf_H_Tool2_Base2.transform.translation.z = H_Tool2_Base2.getOrigin().getZ();
    tf_H_Tool2_Base2.transform.rotation.w = H_Tool2_Base2.getRotation().w();
    tf_H_Tool2_Base2.transform.rotation.x = H_Tool2_Base2.getRotation().x();
    tf_H_Tool2_Base2.transform.rotation.y = H_Tool2_Base2.getRotation().y();
    tf_H_Tool2_Base2.transform.rotation.z = H_Tool2_Base2.getRotation().z();


    // sending transforms to visualize on rviz
    ROS_INFO("Publishing...");
    br.sendTransform(tf_H_Base1_W);
    br.sendTransform(tf_H_Base2_W);
    br.sendTransform(tf_H_Tool1_Base1);
    br.sendTransform(tf_H_Tool2_Base2);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}