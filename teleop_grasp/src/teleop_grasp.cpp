#include <teleop_grasp/teleop_grasp.h>
#include <ros/ros.h>

void
teleop_grasp::test_system()
{
	ROS_INFO_STREAM("node name: " << ros::this_node::getName());
}