#include <ros/ros.h>
#include <teleop_grasp/teleop_grasp.h>

int
main(int argc, char **argv)
{
	// usage:
	// rosrun teleop_grasp demo_teleop_grasp
	
	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "demo_teleop_grasp");
	ros::NodeHandle nh;

	// ------------------------------------------------------------------------------

	// run test function
	teleop_grasp::test_system();

	// exit
	return 0;
}