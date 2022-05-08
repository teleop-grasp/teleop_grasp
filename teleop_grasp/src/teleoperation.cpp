#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "ros_utils/eigen.h"
#include "ros_utils/std.h"
#include "std_msgs/Bool.h"
#include "teleop_grasp/teleop_grasp.h"
#include <cstddef>
#include <iostream>
#include <vector>
#include <franka_msgs/FrankaState.h>

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "teleoperation");
	ros::NodeHandle nh;

	// read node parameters
	auto topic_grasp_state = ros::param::param<std::string>("~topic_grasp_state", "");
	auto topic_pose_hand = ros::param::param<std::string>("~topic_pose_hand", "");
	auto topic_franka_pose_ee = ros::param::param<std::string>("~topic_franka_pose_ee", "");
	auto topic_franka_state = std::string("/franka_state_controller/franka_states");

	// setup subscribers and publishers
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_franka_pose_ee, 1);

	geometry_msgs::Pose pose_hand;
	ros::Subscriber sub_pose_hand = nh.subscribe<geometry_msgs::Pose>(topic_pose_hand, 1, [&](const auto& msg){ pose_hand = *msg; });

	bool gesture_state;
	ros::Subscriber sub_gesture_state = nh.subscribe<std_msgs::Bool>(topic_grasp_state, 1, [&](const auto& msg){ gesture_state = msg->data; });

	// wait for hand psoe
	sleep(5);
	ROS_INFO_STREAM("waiting for hand pose (put up your hand)...");
	ros::topic::waitForMessage<geometry_msgs::Pose>(topic_pose_hand);

	// ----------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("calibrate");

	// scale hand pose
	auto scale_hand_pose = [](const auto& T_hand)
	{
		Eigen::Isometry3d T_hand_scaled = T_hand;
		T_hand_scaled.translation().z() = 2 * T_hand_scaled.translation().z() - 1;
		T_hand_scaled.translation().y() = 2 * T_hand_scaled.translation().y() - 1;
		return T_hand_scaled;
	};

	// read poses and scale initial hand pose
	Eigen::Isometry3d T_hand_init = Eigen::make_tf(*ros::topic::waitForMessage<geometry_msgs::Pose>(topic_pose_hand));
	Eigen::Isometry3d T_ee_init = Eigen::Isometry3d(Eigen::Matrix4d::Map(ros::topic::waitForMessage<franka_msgs::FrankaState>(topic_franka_state)->O_T_EE.data()));
	T_hand_init = scale_hand_pose(T_hand_init);

	ROS_INFO_STREAM("calibrated!");

	// ----------------------------------------------------------------------------------

	ros::Rate rate(100); // hz
	while (ros::ok())
	{
		// process callbacks
		ros::spinOnce();

		// read and scale hand pose
		Eigen::Isometry3d T_hand = Eigen::make_tf(pose_hand);
		T_hand = scale_hand_pose(T_hand);

		// compute difference in hand pose
		Eigen::Matrix3d R_diff = (T_hand_init.inverse() * T_hand).rotation();
		Eigen::Vector3d p_diff = T_hand.translation() - T_hand_init.translation();

		// desired EE pose
		Eigen::Matrix3d R_d = R_diff * T_ee_init.rotation();
		Eigen::Vector3d p_d = T_ee_init.translation() + p_diff;
		Eigen::Isometry3d T_ee_d = Eigen::Translation3d(p_d) * Eigen::Isometry3d(R_d);

		// restrict EE pose
		T_ee_d.translation().y() = std::max(std::min(T_ee_d.translation().y(), 0.4), -0.4);
		T_ee_d.translation().z() = std::max(T_ee_d.translation().z(), 0.3);

		// command EE pose
		static geometry_msgs::PoseStamped pose_d;
		pose_d.pose = geometry_msgs::make_pose(T_ee_d);
		pub_pose.publish(pose_d);

		rate.sleep();
	}

	return 0;
}