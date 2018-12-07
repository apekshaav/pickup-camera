/**
 * This script saves the PSM2 tool tip positions wrt PSM1 tooltip position when a message is sent to the save_data topic.
 * 
 * TODO: save data at the press of a button.
 *
 */

#include "DVRKData.h"

DVRKData::DVRKData()
{
	// set loop rate
	rate = 100; //Hz

	count = 1;

	// set transform between PSM1 base and PSM2 base. This transform is from a previously done calibration and is dependent on the SUJs.
	geometry_msgs::Pose temp;
    temp.position.x = 0.1645;
    temp.position.y = 0.1432;
    temp.position.z = -0.0256;
    temp.orientation.w = 0.6912;
    temp.orientation.x = 0.0498;
    temp.orientation.y = 0.0742;
    temp.orientation.z = 0.7171;
    tf::poseMsgToTF(temp, PSM1_H_PSM2);
	
	ROS_INFO("Initialized!");	

}

DVRKData::~DVRKData()
{
	// clean up
}

// callbacks
void DVRKData::callbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	psm1_tooltip_msg = msg->pose;
  	// ROS_INFO("Position: [%f %f %f] Orientation: [%f %f %f %f]", psm1_tooltip.position.x, psm1_tooltip.position.y, psm1_tooltip.position.z, psm1_tooltip.orientation.w, psm1_tooltip.orientation.x, psm1_tooltip.orientation.y, psm1_tooltip.orientation.z);
}
	
void DVRKData::callbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	psm2_tooltip_msg = msg->pose;
}

void DVRKData::callbackSave(const std_msgs::String::ConstPtr& msg)
{
  	save = true;
}

void DVRKData::startSubscribing()
{
	ROS_INFO("Subscribing...");

	// Starting ROS subscribers
	psm1_sub = n.subscribe("/dvrk/PSM1/position_cartesian_local_current", 1000, &DVRKData::callbackPSM1, this);
	psm2_sub = n.subscribe("/dvrk/PSM2/position_cartesian_local_current", 1000, &DVRKData::callbackPSM2, this);

	save_sub = n.subscribe("/pickup/save", 1000, &DVRKData::callbackSave, this);


	// Starting ROS Publishers
  	save_pub = n.advertise<std_msgs::String>("/pickup/save", 1000);
}

void DVRKData::saveToFile()
{
	ROS_INFO("Point: [%f %f %f]", psm2_tooltip_in_psm1.getX(), psm2_tooltip_in_psm1.getY(), psm2_tooltip_in_psm1.getZ());
	// save to dvrk_data.dat
	// char filename[50];
	// sprintf(buffer, "dvrk_data_%d.dat",count);

	// open file
	// ofstream myFile ("data.bin", ios::out | ios::binary);
    // myFile.write (buffer, 100);
    ROS_INFO("Saved!");
	
}

void DVRKData::run()
{
	ros::Rate loop_rate(rate);

	while(ros::ok())
	{

    	// poll all callbacks
		ros::spinOnce();

		// convert ROS messages to TF pose data
		tf::poseMsgToTF(psm1_tooltip_msg, PSM1_H_PSM1_TOOLTIP);
		tf::poseMsgToTF(psm2_tooltip_msg, psm2_tooltip);

		PSM1_TOOLTIP_H_PSM1 = PSM1_H_PSM1_TOOLTIP.inverse();

		// get required tooltip position for this instant
		psm2_tooltip_in_psm1 = PSM1_TOOLTIP_H_PSM1 * PSM1_H_PSM2 * psm2_tooltip.getOrigin();

		if(save)
		{
			saveToFile();
			count++;
			save = false;
		}
	}
}
