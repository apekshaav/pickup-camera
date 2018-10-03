#include "MovePickup.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "MovePickup");

	MovePickup mv;

	// sleep to allow subscribers to poll published data
	ros::Duration(5).sleep();
	mv.run();

	return 0;
}