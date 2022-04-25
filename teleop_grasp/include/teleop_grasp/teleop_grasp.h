#pragma once

#include <Eigen/Eigen>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"
#include "tf2/LinearMath/Quaternion.h"
#include <array>
#include <franka_gripper/franka_gripper.h>
#include <libavcodec/avcodec.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <string>
#include <ros_utils/geometry_msgs.h>

namespace teleop_grasp
{

	enum class GripperState { CLOSE, OPEN };

	inline bool gesture_state = false;

	inline geometry_msgs::Pose pose_hand;

	inline geometry_msgs::Pose pose_hand_prev;

	inline geometry_msgs::Pose pose_ee_des;

	inline geometry_msgs::Pose pose_ee;

	inline geometry_msgs::Pose pose_d;

	inline std::array<geometry_msgs::Pose, 3> poses_prev;

	void
	calibrate();

	std::string
	get_grasp_topic();

	std::string
	get_pose_topic();

	std::string
	get_franka_state_topic();

	std::string
	get_franka_pose_ee_topic();

	geometry_msgs::Pose
	compute_desired_ee_pose(geometry_msgs::Pose pose_hand);

	geometry_msgs::Pose
	predict_pose(geometry_msgs::Pose current_pose);

	void
	set_pose_robot(geometry_msgs::Pose msg);

	void 
	set_gripper(teleop_grasp::GripperState open_or_close);


	namespace predictor
	{
		geometry_msgs::Pose
		predict_pose_linear(geometry_msgs::Pose current_pose);
	}


}


namespace geometry_msgs
{
	inline geometry_msgs::Pose sub_poses(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to)
	{
		geometry_msgs::Pose result;

		// compute position difference
		result.position.x = to.position.x - from.position.x;
		result.position.y = to.position.y - from.position.y;
		result.position.z = to.position.z - from.position.z;

		// compute orientation difference
		tf2::Quaternion q_from;
		q_from.setW(from.orientation.w);
		q_from.setX(from.orientation.x);
		q_from.setY(from.orientation.y);
		q_from.setZ(from.orientation.z);

		tf2::Quaternion q_to;
		q_to.setW(to.orientation.w);
		q_to.setX(to.orientation.x);
		q_to.setY(to.orientation.y);
		q_to.setZ(to.orientation.z);

		tf2::Quaternion qr = q_to * q_from.inverse();

		result.orientation.w = qr.getW();
		result.orientation.x = qr.getX();
		result.orientation.y = qr.getY();
		result.orientation.z = qr.getZ();

		return result;
	}

	inline geometry_msgs::Pose add_poses(const geometry_msgs::Pose& from, const geometry_msgs::Pose& to)
	{
		geometry_msgs::Pose result;

		// compute position sum
		result.position.x = to.position.x + from.position.x;
		result.position.y = to.position.y + from.position.y;
		result.position.z = to.position.z + from.position.z;

		// compute orientation sum
		tf2::Quaternion q_from;
		q_from.setW(from.orientation.w);
		q_from.setX(from.orientation.x);
		q_from.setY(from.orientation.y);
		q_from.setZ(from.orientation.z);

		tf2::Quaternion q_to;
		q_to.setW(to.orientation.w);
		q_to.setX(to.orientation.x);
		q_to.setY(to.orientation.y);
		q_to.setZ(to.orientation.z);

		tf2::Quaternion qr = q_from; 

		result.orientation.w = qr.getW();
		result.orientation.x = qr.getX();
		result.orientation.y = qr.getY();
		result.orientation.z = qr.getZ();

		return result;
	}
}