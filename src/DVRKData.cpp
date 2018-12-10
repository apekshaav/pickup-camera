/**
 * This script saves the PSM2 tool tip positions wrt PSM1 tooltip position when a message is sent to the save_data topic.
 * 
 * TODO: save data at the press of a button.
 *
 */

#include "DVRKData.h"

typedef struct RobotData
{
	float tipx;
	float tipy;
	float tipz;
} RobotData;

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

    // creating directory to store data
    filePath = "/home/davinci3/catkin_ws_1_6/src/pickup_camera/src/Data/";
    // int status;
	// status = mkdir("~/catkin_ws_1_6/src/pickup_camera/src/Data", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	// ROS_INFO("Status: %d", status);


	// if(mkdir(filePath.c_str(), 2) == -1) //creating a directory
	// {
	// 	ROS_INFO("Error: Unable to create directory");
	// 	exit(1);
	// }

	
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
	
	RobotData out_data;
	out_data.tipx = psm2_tooltip_in_psm1.getX();
	out_data.tipy = psm2_tooltip_in_psm1.getY();
	out_data.tipz = psm2_tooltip_in_psm1.getZ();

	ROS_INFO("Point: [%f %f %f]", out_data.tipx, out_data.tipy, out_data.tipz);
	
	// save to dvrk_data.dat	
	std::string fileName;
	std::string file;	

	// fileName = filePath + std::to_string(count) + "/";

	// if(mkdir(fileName.c_str(), 2) == -1) //creating a directory
	// {
	// 	ROS_INFO("Error: Unable to create directory");
	// 	exit(1);
	// }

	// count++;

	file = filePath + "dvrk_api_" + std::to_string(count) + ".dat";
	ROS_INFO("%s", file.c_str());

	std::ofstream outputStream;

	// outputStream.open(file.c_str(), std::ios::out);	// create file
	// outputStream.close();	

	outputStream.open(file.c_str(), std::ofstream::out | std::ofstream::app); // reopen for editing
	
	outputStream.write((const char*)&out_data.tipx, sizeof(float));
	outputStream.write((const char*)&out_data.tipy, sizeof(float));
	outputStream.write((const char*)&out_data.tipz, sizeof(float));
	// outputStream.write((const char*)&psm2_tooltip_in_psm1.getY(), sizeof(float));
	// outputStream.write((const char*)&psm2_tooltip_in_psm1.getZ(), sizeof(float));
	
	outputStream.close();


	/* some sample code
	std::fstream someFile;
	someFile.open("testBin.dat", std::ios::out | std::ios::in);
	if (someFile.fail())
	{
		// No File Exists
		// cout << "Whoopsie!\n";
		someFile.clear(); // Clear Flag
		someFile.open("testBin.dat", std::ios::out); // Create File
		someFile.close();
		someFile.open("testBin.dat", std::ios::out | std::ios::in); // Reopen for editing
	}

	someFile << "Cheeseburgers are wonderful indeed.";
	someFile.close();
	*/

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
