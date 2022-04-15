#pragma once

#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
namespace teleop_grasp
{
	void
	test_system();
	
	bool
	init();

	void
	gesture_est(const std_msgs::Bool& is_open);

	void
	pose_est(const geometry_msgs::Pose gripper_pose);
}
