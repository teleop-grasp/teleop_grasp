#pragma once

#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include <franka_gripper/franka_gripper.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

namespace teleop_grasp
{

	enum class CMD { CLOSE, OPEN };
	
	bool
	init();

	void
	test_system();

	void
	gesture_est(std_msgs::Bool is_open);

	void
	pose_est(const geometry_msgs::Pose gripper_pose);

	void 
	set_gripper(teleop_grasp::CMD open_or_close);

	void
	follow_franka(const geometry_msgs::PoseConstPtr& msg);

	void
	calibrate();

	// Pose
	// teleop_grasp::predict_motion(Pose);

	// bool
	// gesture_bla(bool a);

	// ros::Publisher  gesture_pub;
	// ros::Publisher  pose;


	teleop_grasp::CMD gripper_state = teleop_grasp::CMD::CLOSE;
}
