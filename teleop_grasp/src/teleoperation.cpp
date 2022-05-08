#include "geometry_msgs/Pose.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "ros_utils/eigen.h"
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
	std::string topic_grasp_state    = ros::param::param<std::string>("~topic_grasp_state", "");
	std::string topic_pose_hand      = ros::param::param<std::string>("~topic_pose_hand", "");
	std::string topic_franka_pose_ee = ros::param::param<std::string>("~topic_franka_pose_ee", "");

	ros::Subscriber sub_grasp        = nh.subscribe<std_msgs::Bool>( topic_grasp_state, 1, [&](const auto& msg){ teleop_grasp::gesture_state = (*msg).data; });
	ros::Subscriber sub_pose_2       = nh.subscribe<geometry_msgs::Pose>( topic_pose_hand, 1, [&](const auto& msg){ teleop_grasp::T_o_hand_2 = *msg;}); 
	ros::Subscriber sub_pose         = nh.subscribe<geometry_msgs::Pose>( topic_pose_hand, 1, [&](const auto& msg){ teleop_grasp::T_o_hand = Eigen::make_tf(*msg);}); 

	// std::cin.get();
	teleop_grasp::calibrate(topic_franka_pose_ee);

	ros::Rate rate(50);

	while (ros::ok()) 
	{

		// ROS_ERROR_STREAM("Pose my dude..." << teleop_grasp::T_o_hand_2);

		// ROS_WARN_STREAM("hand: \n" << geometry_msgs::make_pose(teleop_grasp::T_o_hand));

		// geometry_msgs::Pose a = teleop_grasp::compute_desired_ee_pose();

		// teleop_grasp::command_pose_robot( teleop_grasp::restrict_pose(a));
		// teleop_grasp::command_pose_robot(a);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}