#include "geometry_msgs/Pose.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "teleop_grasp/teleop_grasp.h"
#include <cstddef>
#include <iostream>
#include <vector>

int 
main(int argc, char** argv) 
{
	ros::init(argc, argv, "teleoperation");

	ros::NodeHandle nh;

	// load topics
	auto topic_grasp_state    = ros::param::param<std::string>("~topic_grasp_state","");
	auto topic_pose_hand      = ros::param::param<std::string>("~topic_pose_hand","");
	auto topic_franka_pose_ee = ros::param::param<std::string>("~topic_franka_pose_ee","");

	ros::Subscriber sub_grasp = nh.subscribe<std_msgs::Bool>( topic_grasp_state, 1, [&](const auto& msg){ teleop_grasp::gesture_state = (*msg).data; });
	ros::Subscriber sub_pose  = nh.subscribe<geometry_msgs::Pose>( topic_pose_hand, 1, [&](const auto& msg){ teleop_grasp::pose_hand = *msg;}); 

	// std::cin.get();
	teleop_grasp::calibrate();

	ros::Rate rate(50);

	while (ros::ok()) 
	{

		ROS_WARN_STREAM("received pose: \n" << teleop_grasp::pose_hand);

		// if the hand is open, open the gripper, otherwise close the gripper.
		auto gripper_state = (teleop_grasp::gesture_state) ? teleop_grasp::GripperState::OPEN : teleop_grasp::GripperState::CLOSE;
		teleop_grasp::command_gripper(gripper_state);

		// compute the desired 
		auto des_ee_pose = teleop_grasp::compute_desired_ee_pose();
		teleop_grasp::command_pose_robot(des_ee_pose);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}