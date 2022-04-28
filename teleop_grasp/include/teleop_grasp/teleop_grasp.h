#pragma once

#include <Eigen/Eigen>
#include "Eigen/src/Geometry/Quaternion.h"
#include "Eigen/src/Geometry/Transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"
#include <array>
#include <franka_gripper/franka_gripper.h>
#include <libavcodec/avcodec.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <string>
#include <ros_utils/geometry_msgs.h>
#include "franka/gripper.h"
#include <actionlib/client/simple_action_client.h>
#include "franka_gripper/GraspAction.h"
#include "franka_msgs/FrankaState.h"
#include "ros/publisher.h"
#include "ros/time.h"
#include "ros/topic.h"
#include <ros/ros.h>
#include <ros_utils/eigen.h>
#include <math.h>

namespace teleop_grasp
{

	enum class GripperState { CLOSE, OPEN };
	inline bool                               gesture_state = false;
	inline geometry_msgs::Pose                pose_hand;
	inline geometry_msgs::Pose                pose_hand_prev;
	inline geometry_msgs::Pose                pose_ee_des;
	inline geometry_msgs::Pose                pose_ee;
	inline geometry_msgs::Pose                pose_d;
	inline std::array<geometry_msgs::Pose, 3> poses_prev;
	inline constexpr int                      MAX_LIN_VELOCITY = 1;
	inline constexpr double                   MAX_ANG_VELOCITY = M_PI / 2.0;
	inline constexpr double                   CHANGE_THRESHOLD = 1e-6;

	void
	calibrate();

	std::string 
	get_topic(const std::string& topic);

	geometry_msgs::Pose
	compute_desired_ee_pose(const geometry_msgs::Pose& pose_hand);

	void
	command_pose_robot(const geometry_msgs::Pose& msg);

	void 
	command_gripper(const teleop_grasp::GripperState& open_or_close);


	namespace predictor
	{
		geometry_msgs::Pose
		predict_pose_linear();
	}
}
/*
<param name="topic_grasp_state" value="$(arg grasp_state)"/>
<param name="topic_image_raw" value="$(arg image_raw)" />
*/
