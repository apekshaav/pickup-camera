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

geometry_msgs::Pose rec_msg_1, rec_msg_2, rec_msg_3;

void callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  rec_msg_1 = msg->pose;
}

void callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  rec_msg_2 = msg->pose;
}

void callbackPSM3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  rec_msg_3 = msg->pose;
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "visualize");
  ros::NodeHandle n;

  ros::Subscriber sub_1 = n.subscribe("/dvrk/PSM1/position_cartesian_current", 1000, callbackPSM1);
  ros::Subscriber sub_2 = n.subscribe("/dvrk/PSM2/position_cartesian_current", 1000, callbackPSM2);
  ros::Subscriber sub_3 = n.subscribe("/dvrk/PSM2/position_cartesian_current", 1000, callbackPSM3);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    

    //////////////////////////////////////////////////////////////////////////////////
    // Transforms                                                                   //
    // ----------                                                                   //
    // H_Base1_W      : Base 1 to World                                             //
    // H_Base2_W      : Base 2 to World                                             //
    // H_Base3_W      : Base 3 to World                                             //
    // H_Tool1_Base1  : Tooltip 1 to Base 1                                         //
    // H_Tool2_Base2  : Tooltip 2 to Base 2                                         //
    // H_Base1_Tool1  : Base 1 to Tooltip 1                                         //
    // H_Base2_Tool2  : Base 2 to Tooltip 2                                         //
    // H_Tool1_Cam    : Tooltip 1 to Pick-Up Camera                                 //
    // H_Cam          : Final Transform - Base 2 to Pick-Up Camera                  //
    // H_ECM_W        : ECM tip to World
    //////////////////////////////////////////////////////////////////////////////////

    // creating object to broadcast transforms 
    static tf2_ros::TransformBroadcaster br;

    // for H_Base1_W (base frame of PSM1 wrt world)
    geometry_msgs::TransformStamped tf_H_Base1_W;
  
    tf_H_Base1_W.header.stamp = ros::Time::now();
    tf_H_Base1_W.header.frame_id = "world";
    tf_H_Base1_W.child_frame_id = "base1";
    tf_H_Base1_W.transform.translation.x = -0.0962;
    tf_H_Base1_W.transform.translation.y = 0.2701;
    tf_H_Base1_W.transform.translation.z = 0.2053;
    tf_H_Base1_W.transform.rotation.w = 0.3515;
    tf_H_Base1_W.transform.rotation.x = -0.8605;
    tf_H_Base1_W.transform.rotation.y = -0.3373;
    tf_H_Base1_W.transform.rotation.z = -0.1489;
    
  
    // for H_Base2_W (base frame of PSM2 wrt world)
    geometry_msgs::TransformStamped tf_H_Base2_W;
  
    tf_H_Base2_W.header.stamp = ros::Time::now();
    tf_H_Base2_W.header.frame_id = "world";
    tf_H_Base2_W.child_frame_id = "base2";
    tf_H_Base2_W.transform.translation.x = 0.0982;
    tf_H_Base2_W.transform.translation.y = 0.0876;
    tf_H_Base2_W.transform.translation.z = 0.1109;
    tf_H_Base2_W.transform.rotation.w = 0.2722;
    tf_H_Base2_W.transform.rotation.x = -0.7838;
    tf_H_Base2_W.transform.rotation.y = 0.5167;
    tf_H_Base2_W.transform.rotation.z = 0.2111;

    // for H_Base2_W (base frame of PSM3 wrt world)
    geometry_msgs::TransformStamped tf_H_Base3_W;
  
    tf_H_Base3_W.header.stamp = ros::Time::now();
    tf_H_Base3_W.header.frame_id = "world";
    tf_H_Base3_W.child_frame_id = "base3";
    tf_H_Base3_W.transform.translation.x = -0.1228;
    tf_H_Base3_W.transform.translation.y = 0.2851;
    tf_H_Base3_W.transform.translation.z = -0.1305;
    tf_H_Base3_W.transform.rotation.w = 0.3018;
    tf_H_Base3_W.transform.rotation.x = -0.7556;
    tf_H_Base3_W.transform.rotation.y = -0.5455;
    tf_H_Base3_W.transform.rotation.z = -0.2009;


    // for H_ECM_W (tip of ECM wrt world)
    geometry_msgs::TransformStamped tf_H_ECM_W;
  
    tf_H_ECM_W.header.stamp = ros::Time::now();
    tf_H_ECM_W.header.frame_id = "world";
    tf_H_ECM_W.child_frame_id = "ECM";
    tf_H_ECM_W.transform.translation.x = -0.0564;
    tf_H_ECM_W.transform.translation.y = 1.2975;
    tf_H_ECM_W.transform.translation.z = 0.9983;
    tf_H_ECM_W.transform.rotation.w = 0.3627;
    tf_H_ECM_W.transform.rotation.x = 0.9316;
    tf_H_ECM_W.transform.rotation.y = -0.0173;
    tf_H_ECM_W.transform.rotation.z = -0.0182;
      

    // using data retrieved from subscriber
    tf::Pose H_Tool1_Base1;
    tf::poseMsgToTF(rec_msg_1, H_Tool1_Base1);
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


    // using data retrieved from subscriber
    tf::Pose H_Tool3_Base3;
    tf::poseMsgToTF(rec_msg_3, H_Tool3_Base3);
    geometry_msgs::TransformStamped tf_H_Tool3_Base3;
  
    tf_H_Tool3_Base3.header.stamp = ros::Time::now();
    tf_H_Tool3_Base3.header.frame_id = "base3";
    tf_H_Tool3_Base3.child_frame_id = "tool3";
    tf_H_Tool3_Base3.transform.translation.x = H_Tool3_Base3.getOrigin().getX();
    tf_H_Tool3_Base3.transform.translation.y = H_Tool3_Base3.getOrigin().getY();
    tf_H_Tool3_Base3.transform.translation.z = H_Tool3_Base3.getOrigin().getZ();
    tf_H_Tool3_Base3.transform.rotation.w = H_Tool3_Base3.getRotation().w();
    tf_H_Tool3_Base3.transform.rotation.x = H_Tool3_Base3.getRotation().x();
    tf_H_Tool3_Base3.transform.rotation.y = H_Tool3_Base3.getRotation().y();
    tf_H_Tool3_Base3.transform.rotation.z = H_Tool3_Base3.getRotation().z();


    // sending transforms to visualize on rviz
    ROS_INFO("Publishing...");
    br.sendTransform(tf_H_Base1_W);
    br.sendTransform(tf_H_Base2_W);
    br.sendTransform(tf_H_Base3_W);
    br.sendTransform(tf_H_ECM_W);
    br.sendTransform(tf_H_Tool1_Base1);
    br.sendTransform(tf_H_Tool2_Base2);
    br.sendTransform(tf_H_Tool3_Base3);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}