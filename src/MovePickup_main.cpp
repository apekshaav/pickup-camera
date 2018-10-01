#include "MovePickup.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "MovePickup");

	MovePickup mv;
	mv.run();

	return 0;
}