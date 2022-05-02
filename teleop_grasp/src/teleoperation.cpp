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
	auto topic_grasp_state      = ros::param::param<std::string>("~topic_grasp_state","");
	auto topic_pose_hand        = ros::param::param<std::string>("~topic_pose_hand","");
	auto topic_franka_pose_ee   = ros::param::param<std::string>("~topic_franka_pose_ee","");

	ros::Subscriber sub_grasp = nh.subscribe<std_msgs::Bool>( topic_grasp_state, 1, [&](const auto& msg){ teleop_grasp::gesture_state = (*msg).data; });
	ros::Subscriber sub_pose  = nh.subscribe<geometry_msgs::Pose>( topic_pose_hand, 1, [&](const auto& msg){ teleop_grasp::pose_hand = *msg; }); 

	std::cin.get();
	teleop_grasp::calibrate();

	ros::Rate rate(10);

	while (ros::ok()) 
	{

		// open and close gripper
		auto gripper_state = (teleop_grasp::gesture_state) ? teleop_grasp::GripperState::OPEN : teleop_grasp::GripperState::CLOSE;
		teleop_grasp::command_gripper( gripper_state );

		// predict the next position of hand
		auto pose_ee_pred = teleop_grasp::predictor::predict_pose_linear();

		// compute the new position based on relative difference between hand poses
		teleop_grasp::pose_ee_des = teleop_grasp::compute_desired_ee_pose( teleop_grasp::pose_hand );

		// send pose to franka | choose between predicted pose, and desired pose
		teleop_grasp::command_pose_robot( teleop_grasp::pose_ee_des );
		// teleop_grasp::command_pose_robot( teleop_grasp::pose_ee_des, topic_franka_pose_ee );

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}