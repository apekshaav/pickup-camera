#include "MovePickup.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "MovePickup");

	MovePickup mv;

	ros::Duration(1).sleep();
	mv.run();

	return 0;
}