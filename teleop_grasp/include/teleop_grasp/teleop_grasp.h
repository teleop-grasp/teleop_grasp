#pragma once

#include <Eigen/Eigen>
#include "Eigen/src/Geometry/Transform.h"
#include "geometry_msgs/Pose.h"
#include <array>
#include <franka_gripper/franka_gripper.h>
#include <libavcodec/avcodec.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <string>
#include <ros_utils/geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

namespace teleop_grasp
{

	enum class GripperState { CLOSE, OPEN };
	inline Eigen::Isometry3d                  T_o_ee_cal;
	inline bool                               gesture_state      = false;
	inline bool                               gesture_state_prev = false;
	inline Eigen::Isometry3d                  T_o_hand;
	inline geometry_msgs::Pose                T_o_hand_2;
	inline Eigen::Isometry3d                  T_o_hand_prev;
	inline Eigen::Isometry3d                  T_o_ee_des;
	inline Eigen::Isometry3d                  T_o_ee;
	inline constexpr double                   MAX_LIN_VELOCITY   = 0.5;
	inline constexpr double                   MAX_ANG_VELOCITY   = M_PI / 2.0;
	inline constexpr double                   CHANGE_THRESHOLD   = 0.05;

	void
	calibrate(const std::string& topic_franka_pose_ee);

	geometry_msgs::Pose
	compute_desired_ee_pose();

	void
	command_pose_robot(const geometry_msgs::Pose& msg, const std::string& topic_pose = "/cartesian_pd_nullspace_controller/command");

	void 
	command_gripper(const teleop_grasp::GripperState& open_or_close);


	namespace predictor
	{
		geometry_msgs::Pose
		predict_pose_linear();
	}

	Eigen::Isometry3d
	get_current_hand_pose();

	Eigen::Isometry3d
	get_current_franka_pose();

	void
	set_current_franka_pose(const geometry_msgs::Pose &des_pose);

	bool
	has_too_much_change_occurred(const geometry_msgs::Pose& pose);

	Eigen::Isometry3d
	restrict_pose(Eigen::Isometry3d pose);

}
