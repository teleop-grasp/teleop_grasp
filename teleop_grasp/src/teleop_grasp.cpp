#include "franka_gripper/MoveAction.h"
#include "franka_msgs/FrankaState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros_utils/geometry_msgs.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <teleop_grasp/teleop_grasp.h>
#include <ros_utils/eigen.h>

void
teleop_grasp::calibrate(const std::string& topic_franka_pose_ee)
{
	static ros::NodeHandle nh;

	// get robot ee pose
	teleop_grasp::T_o_ee_cal = get_current_franka_pose();

	// get hand pose
	teleop_grasp::T_o_hand = get_current_hand_pose();

	// franka take init pose
	command_pose_robot( geometry_msgs::make_pose(teleop_grasp::T_o_ee_cal), topic_franka_pose_ee);

	// calibration completed
	ROS_WARN_STREAM("calibration has been completed...");
}

geometry_msgs::Pose
teleop_grasp::compute_desired_ee_pose()
{
	teleop_grasp::T_o_hand_prev = teleop_grasp::T_o_hand;
	teleop_grasp::T_o_hand      = get_current_hand_pose();
	teleop_grasp::T_o_ee        = get_current_franka_pose();

	teleop_grasp::T_o_hand = teleop_grasp::restrict_pose(teleop_grasp::T_o_hand);

	// ROS_ERROR_STREAM("T hand: \n" << geometry_msgs::make_pose(T_o_hand));
	// ROS_ERROR_STREAM("T hand prev: \n" << geometry_msgs::make_pose(T_o_hand_prev));

	Eigen::Isometry3d T_hand_prev_hand = teleop_grasp::T_o_hand * T_o_hand_prev.inverse();
	// Eigen::Isometry3d T_hand_prev_hand = T_o_hand_prev.inverse() * teleop_grasp::T_o_hand;

	// T_hand_prev_hand.translation() *= -1; // fixing translation direction

	ROS_ERROR_STREAM("dT: \n" << geometry_msgs::make_pose(T_hand_prev_hand));
	ROS_ERROR_STREAM("det R pos : \n" << teleop_grasp::T_o_hand.rotation().determinant());
	

	// if the velocity becomes too large, often meaning the hand has moved off screen.
	if ( has_too_much_change_occurred(geometry_msgs::make_pose(T_hand_prev_hand))  )
	{
		ROS_ERROR_STREAM("hand lost...");
		return geometry_msgs::make_pose(teleop_grasp::T_o_ee);
	}

	// Eigen::Isometry3d T_ee_des = T_hand_prev_hand * teleop_grasp::T_o_ee;
	Eigen::Isometry3d T_ee_des = teleop_grasp::T_o_ee * T_hand_prev_hand;

	// ROS_ERROR_STREAM("teleop_grasp::T_o_ee: \n" << geometry_msgs::make_pose(teleop_grasp::T_o_ee));
	// ROS_ERROR_STREAM("T_hand_prev_hand: \n" << geometry_msgs::make_pose(T_hand_prev_hand));
	// ROS_ERROR_STREAM("T des: \n" << geometry_msgs::make_pose(T_ee_des));


	// ROS_ERROR_STREAM("restricted pose: \n" << teleop_grasp::restrict_pose(geometry_msgs::make_pose(T_ee_des)));

	return geometry_msgs::make_pose( T_ee_des );
	// return teleop_grasp::restrict_pose( geometry_msgs::make_pose( T_ee_des ) );
}

void
teleop_grasp::command_pose_robot(const geometry_msgs::Pose& pose, const std::string& topic_pose )
{

	static ros::NodeHandle nh;

	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,1);

	// convert msg to correst geometry_msgs::PoseStamped type
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = pose;
	pose_stamped.header.stamp.sec = ros::Time().sec;

	// send message
	pub_pose.publish(pose_stamped);
}

void
teleop_grasp::command_gripper(const bool& open_or_close)
{

	actionlib::SimpleActionClient<franka_gripper::MoveAction> action("/franka_gripper/move",true);
	action.waitForServer();

	franka_gripper::MoveAction msg;
	msg.action_goal.goal.speed = 0.1;

	if (open_or_close == teleop_grasp::gesture_state_prev )
		return;

	if (open_or_close == bool(teleop_grasp::GripperState::OPEN))
	{
		teleop_grasp::gesture_state_prev = true;
		msg.action_goal.goal.width = 0.045;
		action.sendGoal(msg.action_goal.goal);
	}
	else
	{
		teleop_grasp::gesture_state_prev = false;
		msg.action_goal.goal.width = 0.01;
		action.sendGoal(msg.action_goal.goal);
	}
}

geometry_msgs::Pose
teleop_grasp::predictor::predict_pose_linear()
{
	// compute diff transform from prev hand pose to current hand pose
	Eigen::Isometry3d d_pose = teleop_grasp::T_o_hand_prev.inverse() * teleop_grasp::T_o_hand;

	// extend relative prediction vector
	d_pose.translation() *= 2;

	// compute new desired ee pose
	teleop_grasp::T_o_ee_des = teleop_grasp::T_o_ee * d_pose;

	// convert to geometry_msgs::Pose and return
	return geometry_msgs::make_pose(teleop_grasp::T_o_ee_des);
}

Eigen::Isometry3d
teleop_grasp::get_current_hand_pose()
{
	static ros::NodeHandle nh;
	auto topic_pose_hand = ros::param::param<std::string>("/teleoperation/topic_pose_hand","");
	return Eigen::make_tf(*(ros::topic::waitForMessage<geometry_msgs::Pose>(topic_pose_hand,nh)));
}

Eigen::Isometry3d
teleop_grasp::get_current_franka_pose()
{
	static ros::NodeHandle nh;
	auto franka_state     = *(ros::topic::waitForMessage<franka_msgs::FrankaState>("/franka_state_controller/franka_states",nh));
	return Eigen::Isometry3d(Eigen::Matrix4d::Map(franka_state.O_T_EE.data()));
}

void
teleop_grasp::set_current_franka_pose(const geometry_msgs::Pose &des_pose)
{
	static ros::NodeHandle nh;
	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_admittance_controller/command",1);

	// convert msg to correst geometry_msgs::PoseStamped type
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = des_pose;
	pose_stamped.header.stamp.sec = ros::Time().sec;

	// send message
	pub_pose.publish(pose_stamped);
}

bool
teleop_grasp::has_too_much_change_occurred(const geometry_msgs::Pose& d_pose)
{
	// ROS_WARN_STREAM(" d_pose bad?! " << d_pose);
	if ( std::abs(d_pose.position.x) > teleop_grasp::MAX_LIN_VELOCITY or std::abs(d_pose.position.y) > teleop_grasp::MAX_LIN_VELOCITY) //or std::abs(d_pose.position.z) > teleop_grasp::MAX_LIN_VELOCITY)
		return true;
	else
		return false;
}

Eigen::Isometry3d
teleop_grasp::restrict_pose(Eigen::Isometry3d pose)
{

	geometry_msgs::Pose pose_p = geometry_msgs::make_pose(pose);
	double min = -1; // m
	double max =  1; // m

	ROS_WARN_STREAM("pose pre restrict" << pose_p);
	
	pose_p.position.z = pose_p.position.z - 0.5;
	pose_p.position.y = pose_p.position.y - 0.5;



	// pose.position.x = pose.position.x * (std::abs(min) + std::abs(max)) - max;
	// pose.position.y = pose.position.y * (std::abs(min) + std::abs(max)) - max;

	// pose.position.x = pose.position.x + 0.5;
	// pose.position.z = pose.position.z + 0.2;

	pose_p.position.z = std::max(pose_p.position.z,0.5);

	ROS_WARN_STREAM("pose post restrict" << pose_p);

	return pose;
}