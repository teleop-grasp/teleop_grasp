#include "actionlib/client/client_helpers.h"
#include "actionlib/client/simple_action_client.h"
#include "franka/gripper.h"
#include "franka_gripper/GraspAction.h"
#include "franka_gripper/HomingAction.h"
#include "franka_gripper/MoveAction.h"
#include "franka_gripper/MoveActionGoal.h"
#include "franka_gripper/MoveGoal.h"
#include "franka_msgs/FrankaState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/time.h"
#include "ros/topic.h"
#include "ros_utils/geometry_msgs.h"
#include <boost/regex/v4/syntax_type.hpp>
#include <cstddef>
#include <ctime>
#include <string>
#include <teleop_grasp/teleop_grasp.h>
#include <ros/ros.h>

void 
teleop_grasp::calibrate()
{
	static ros::NodeHandle nh;

	// get robot TCP pose
	auto franka_state     = *(ros::topic::waitForMessage<franka_msgs::FrankaState>(get_franka_state_topic(),nh));
	teleop_grasp::pose_ee = geometry_msgs::make_pose(Eigen::Isometry3d(Eigen::Matrix4d::Map(franka_state.O_T_EE.data())));

	// get hand pose
	// teleop_grasp::pose_hand = *(ros::topic::waitForMessage<geometry_msgs::Pose>(get_pose_topic(),nh));
	teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand;

	// calibration completed
	ROS_INFO_STREAM("calibration has been completed with:\n\trobot init tcp pose: " << teleop_grasp::pose_ee << "\n\thand init pose: " << teleop_grasp::pose_hand);
}

std::string
teleop_grasp::get_grasp_topic()
{
	static ros::NodeHandle nh;
	std::string grasp = "";
	if (not ros::param::get("/teleop_grasp/topic_grasp_hand", grasp)) 
	{
		ROS_ERROR_STREAM("Could not read parameter grasp");
		return grasp;
	}
	return grasp;
}

std::string 
teleop_grasp::get_pose_topic()
{
	static ros::NodeHandle nh;
	std::string pose = "";
	if (not ros::param::get("/teleop_grasp/topic_pose_hand", pose)) 
	{
		ROS_ERROR_STREAM("Could not read parameter pose");
		return pose;
	}
	return pose;
}

std::string 
teleop_grasp::get_franka_state_topic()
{
	static ros::NodeHandle nh;
	std::string topic_franka_state = "";
	if (not ros::param::get("/teleop_grasp/topic_franka_state", topic_franka_state)) 
	{
		ROS_ERROR_STREAM("Could not read parameter topic_franka_state");
		return topic_franka_state;
	}
	return topic_franka_state;
}

std::string
teleop_grasp::get_franka_pose_ee_topic()
{
	static ros::NodeHandle nh;
	std::string topic_franka_pose_ee = "";
	ROS_WARN_STREAM("get_franka_pose_ee_topic...");
	if (not ros::param::get("/teleop_grasp/topic_franka_pose_ee", topic_franka_pose_ee)) 
	{
		ROS_ERROR_STREAM("Could not read parameter topic_franka_pose_ee");
		return topic_franka_pose_ee;
	}
	return topic_franka_pose_ee;	
}

geometry_msgs::Pose
teleop_grasp::compute_desired_ee_pose(geometry_msgs::Pose pose_hand)
{
	auto d_pose = geometry_msgs::sub_poses(pose_hand, teleop_grasp::pose_hand_prev);

	auto pose_ee_des = geometry_msgs::add_poses( teleop_grasp::pose_ee, d_pose );

	return pose_ee_des;
}

void
teleop_grasp::set_pose_robot(geometry_msgs::Pose pose)
{
	static ros::NodeHandle nh;
	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(teleop_grasp::get_franka_pose_ee_topic(),1);

	geometry_msgs::PoseStamped pose_stamped;

	pose_stamped.pose = pose;
	pose_stamped.header.stamp.sec = ros::Time().sec;

	pub_pose.publish(pose_stamped);
}

void
teleop_grasp::set_gripper(teleop_grasp::GripperState open_or_close)
{
	actionlib::SimpleActionClient<franka_gripper::GraspAction> action("/franka_gripper/grasp", true);
	
	ROS_WARN("Waiting for action server to start..."); 

	bool success = action.isServerConnected();

	if (success) 
		ROS_WARN("Connected successfully!...");

	action.waitForServer();

	ROS_WARN("Action server started, sending goal...");

	franka_gripper::GraspAction msg;

	msg.action_goal.goal.speed = 0.1;

	if (open_or_close == teleop_grasp::GripperState::OPEN)
	{
		msg.action_goal.goal.width = 0.045;
		action.sendGoal(msg.action_goal.goal);
	}
	else
	{
		msg.action_goal.goal.width = 0.01;
		action.sendGoal(msg.action_goal.goal);
	}
}