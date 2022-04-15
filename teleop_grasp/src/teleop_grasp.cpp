#include <teleop_grasp/teleop_grasp.h>
#include <ros/ros.h>

void
teleop_grasp::test_system()
{
	ROS_INFO_STREAM("node name: " << ros::this_node::getName());
}

bool
teleop_grasp::init()
{
	return false;
}

void
teleop_grasp::gesture_est(const std_msgs::Bool& is_open)
{

}

void
teleop_grasp::pose_est(const geometry_msgs::Pose gripper_pose)
{
	
}