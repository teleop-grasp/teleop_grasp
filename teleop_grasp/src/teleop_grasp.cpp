#include "actionlib/client/client_helpers.h"
#include "actionlib/client/simple_action_client.h"
#include "franka/gripper.h"
#include "franka_gripper/GraspAction.h"
#include "franka_gripper/HomingAction.h"
#include "franka_gripper/MoveAction.h"
#include "franka_gripper/MoveActionGoal.h"
#include "franka_gripper/MoveGoal.h"
#include <teleop_grasp/teleop_grasp.h>
#include <ros/ros.h>

bool
teleop_grasp::init()
{
	return false;
}

void 
teleop_grasp::test_system()
{
	return;
}

void
teleop_grasp::pose_est(const geometry_msgs::Pose gripper_pose)
{
	return;
}

void
teleop_grasp::set_gripper(teleop_grasp::CMD open_or_close)
{
	// in case gripper state is the same as requested
	if (gripper_state == open_or_close) { ROS_WARN_STREAM("The requested teleop_grasp::CMD is already present..."); return; }

	actionlib::SimpleActionClient<franka_gripper::GraspAction> action("/franka_gripper/grasp", true);
	
	ROS_WARN("Waiting for action server to start..."); 

	bool success = action.isServerConnected();

	if (success) 
		ROS_WARN("Connected successfully!...");

	action.waitForServer();

	ROS_WARN("Action server started, sending goal...");

	franka_gripper::GraspAction msg;

	msg.action_goal.goal.speed = 0.1;

	if (open_or_close == teleop_grasp::CMD::OPEN)
	{
		gripper_state = CMD::OPEN;
		msg.action_goal.goal.width = 0.045;
		action.sendGoal(msg.action_goal.goal);
	}
	else
	{
		gripper_state = CMD::CLOSE;
		msg.action_goal.goal.width = 0.01;
		action.sendGoal(msg.action_goal.goal);
	}
}