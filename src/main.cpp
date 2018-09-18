#include "ReferenceFrame.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "setReferenceFrame");

	ReferenceFrame rf;
	rf.run();

	return 0;
}