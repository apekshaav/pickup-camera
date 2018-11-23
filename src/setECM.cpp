/**
 * This script is used to send the transform (position, quaternion) to set the base transform 
 * of the two da Vinci arms PSM1 (Yellow Arm) and PSM2 (Yellow Arm) with respect to the endoscope (ECM tip coordinate frame).
 */

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// TF
#include <tf/tf.h>

geometry_msgs::Pose rec_pose;

void callbackECM(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  // storing only pose not timestamp details
  rec_pose = msg->pose;

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "setECM");
  ros::NodeHandle n;

  // subscribers
  ros::Subscriber ECM_sub = n.subscribe("/dvrk/ECM/position_cartesian_local_current", 1000, callbackECM);
  
  // publishers
  ros::Publisher PSM1_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_base_frame", 1000);
  ros::Publisher PSM2_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM2/set_base_frame", 1000);
  // ros::Publisher psm3_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM3/set_base_frame", 1000);
  
  ros::Publisher MTML_PSM2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM2/set_registration_rotation", 1000);
  ros::Publisher MTMR_PSM1_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM1/set_registration_rotation", 1000);
  // ros::Publisher MTMR_PSM2_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTMR_PSM2/set_registration_rotation", 1000);
  // ros::Publisher MTML_PSM3_rot_pub = n.advertise<geometry_msgs::Quaternion>("/dvrk/MTML_PSM3/set_registration_rotation", 1000);
 
  // initialize

  // transform from PSM1 Base Frame to ECM base frame
  geometry_msgs::Pose temp1;
  temp1.position.x = -0.1295;
  temp1.position.y = -0.1103;
  temp1.position.z = -0.0706;
  temp1.orientation.w = 0.8736;
  temp1.orientation.x = 0.3466;
  temp1.orientation.y = 0.0983;
  temp1.orientation.z = -0.3272;

  tf::Pose ECM_Base_H_PSM1_Base;
  tf::poseMsgToTF(temp1, ECM_Base_H_PSM1_Base);

  // transform from PSM2 Base Frame to ECM base frame
  geometry_msgs::Pose temp2;
  temp2.position.x = 0.0896;
  temp2.position.y = -0.0979;
  temp2.position.z = -0.0776;
  temp2.orientation.w = 0.8139;
  temp2.orientation.x = 0.3779;
  temp2.orientation.y = -0.1322;
  temp2.orientation.z = 0.4212;

  tf::Pose ECM_Base_H_PSM2_Base;
  tf::poseMsgToTF(temp2, ECM_Base_H_PSM2_Base);

  // setting registration rotation of MTML-PSM2
  geometry_msgs::Quaternion q2;
  q2.w = 0;
  q2.x = 0;
  q2.y = 1; 
  q2.z = 0;

  // setting registration rotation of MTMR-PSM1
  geometry_msgs::Quaternion q1;
  q1.w = 0;
  q1.x = 0;
  q1.y = 1; 
  q1.z = 0;

  ros::Rate loop_rate(100);
  
  int baseFrameCount = 0;
  int regRotFlag = 0;
  while (ros::ok())
  {
   
    // subscribers poll data here
    ros::spinOnce();

    // ROS_INFO("PSM1: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp1.position.x, temp1.position.y, temp1.position.z, temp1.orientation.w, temp1.orientation.x, temp1.orientation.y, temp1.orientation.z);
    // ROS_INFO("PSM2: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp2.position.x, temp2.position.y, temp2.position.z, temp2.orientation.w, temp2.orientation.x, temp2.orientation.y, temp2.orientation.z);

    // udpate ECM tip data retrieved from subscriber
    tf::Pose ECM_Base_H_ECM_Tip;
    tf::poseMsgToTF(rec_pose, ECM_Base_H_ECM_Tip);
    // ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", rec_pose.position.x, rec_pose.position.y, rec_pose.position.z, rec_pose.orientation.w, rec_pose.orientation.x, rec_pose.orientation.y, rec_pose.orientation.z);
    tf::Transform ECM_Tip_H_ECM_Base = ECM_Base_H_ECM_Tip.inverse();
    geometry_msgs::Pose test;
    quaternionTFToMsg(ECM_Tip_H_ECM_Base.getRotation(), test.orientation);
    // ROS_INFO("Inv: Position: [%f %f %f] Orientation: [%f %f %f %f]", ECM_Tip_H_ECM_Base.getOrigin().getX(), ECM_Base_H_ECM_Tip.getOrigin().getY(), ECM_Base_H_ECM_Tip.getOrigin().getZ(), test.orientation.w, test.orientation.x, test.orientation.y, test.orientation.z);

    // transformation chains for PSM1,2
    tf::Pose ECM_Tip_H_PSM1_Base = ECM_Tip_H_ECM_Base * ECM_Base_H_PSM1_Base;
    tf::Pose ECM_Tip_H_PSM2_Base = ECM_Tip_H_ECM_Base * ECM_Base_H_PSM2_Base;

    // converting to geometry msgs to send to ROS topic
    // for PSM1
    geometry_msgs::Pose msg1;
    tf::poseTFToMsg(ECM_Tip_H_PSM1_Base, msg1);
    // for PSM2
    geometry_msgs::Pose msg2;
    tf::poseTFToMsg(ECM_Tip_H_PSM2_Base, msg2);

    // ROS_INFO("Chain 1: Position: [%f %f %f] Orientation: [%f %f %f %f]", msg1.position.x, msg1.position.y, msg1.position.z, msg1.orientation.w, msg1.orientation.x, msg1.orientation.y, msg1.orientation.z);
    // ROS_INFO("Chain 2: Position: [%f %f %f] Orientation: [%f %f %f %f]", msg2.position.x, msg2.position.y, msg2.position.z, msg2.orientation.w, msg2.orientation.x, msg2.orientation.y, msg2.orientation.z);

    if (baseFrameCount%100==0)
      ROS_INFO("ECM: Updating base frames...");
    baseFrameCount++;

    //clearing and resetting every 1000 times
    if (baseFrameCount==1000)
        baseFrameCount = 0;

    PSM1_pub.publish(msg1);
    PSM2_pub.publish(msg2);

    // publishing registration rotations
    MTML_PSM2_rot_pub.publish(q2);
    MTMR_PSM1_rot_pub.publish(q1);

    if(regRotFlag==0)
    {
    	ROS_INFO("Setting registration rotation for both teleop pairs");
      regRotFlag = 1;
    }


    loop_rate.sleep();
  }


  return 0;
}
