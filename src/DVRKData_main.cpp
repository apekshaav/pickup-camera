#include "DVRKData.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "stream");

	DVRKData stream;

	stream.startSubscribing();
	
	// sleep to allow subscribers to poll published data
	ros::Duration(1).sleep();
	
	stream.run();

	return 0;
}