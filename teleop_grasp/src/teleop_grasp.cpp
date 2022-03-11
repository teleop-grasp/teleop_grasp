#include <teleop_grasp/teleop_grasp.h>

#include <string>
#include <ros/ros.h>

std::string
teleop_grasp::test_system()
{
	ROS_INFO_STREAM("node name: " << ros::this_node::getName());
}