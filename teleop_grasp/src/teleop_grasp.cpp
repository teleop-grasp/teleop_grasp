#include "franka_gripper/MoveAction.h"
#include "franka_msgs/FrankaState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros_utils/geometry_msgs.h"
#include <cmath>
#include <teleop_grasp/teleop_grasp.h>
#include <ros_utils/eigen.h>

void 
teleop_grasp::calibrate()
{

	static ros::NodeHandle nh;

	// get robot ee pose
	teleop_grasp::calibrate_pose_franka = utils::get_current_franka_pose();

	// get hand pose
	teleop_grasp::pose_hand = utils::get_current_hand_pose();

	ROS_WARN_STREAM("calibrated position: \n" << teleop_grasp::calibrate_pose_franka);

	auto T1 = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
	auto T2 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

	teleop_grasp::calibrate_pose_franka.orientation = geometry_msgs::make_pose( Eigen::make_tf(teleop_grasp::calibrate_pose_franka) * T1 * T2 ).orientation;

	command_pose_robot(teleop_grasp::calibrate_pose_franka);

	// calibration completed
	ROS_WARN_STREAM("calibration has been completed...");
}

geometry_msgs::Pose
teleop_grasp::compute_desired_ee_pose()
{

	// auto pose_hand_tf      = Eigen::make_tf(teleop_grasp::pose_hand);
	// auto pose_hand_prev_tf = Eigen::make_tf(teleop_grasp::pose_hand_prev);

	// // compute diff transform from prev hand pose to current hand pose
	// auto d_pose_tf = pose_hand_prev_tf.inverse() * pose_hand_tf;

	// // if the velocity becomes too large
	// if ( utils::has_too_much_change_occurred(pose_hand_prev_tf, pose_hand_tf)  )
	// {
	// 	ROS_WARN_STREAM("velocity too large...");
	// 	d_pose_tf = Eigen::Isometry3d::Identity();
	// 	return geometry_msgs::make_pose( pose_hand_tf );
	// }

	// // if no changes are made to the pose, this is caused by the hand moving outside the camera's view
	// if ( utils::has_any_change_occurred(pose_hand_prev_tf, pose_hand_tf) )
	// {
	// 	ROS_WARN_STREAM("no changes are made...");
	// 	d_pose_tf = Eigen::Isometry3d::Identity();
	// 	return geometry_msgs::make_pose( pose_hand_tf );
	// }

	auto T1 = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
	auto T2 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

	teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand;

	teleop_grasp::pose_hand.orientation = geometry_msgs::make_pose( Eigen::make_tf( teleop_grasp::calibrate_pose_franka ) ).orientation;
	teleop_grasp::pose_hand = teleop_grasp::utils::restrict_pose(teleop_grasp::pose_hand);

	ROS_WARN_STREAM("finished hand pose = \n" << teleop_grasp::pose_hand);

	return teleop_grasp::pose_hand;
}

void
teleop_grasp::command_pose_robot(const geometry_msgs::Pose& pose, const std::string& topic_pose )
{
	static ros::NodeHandle nh;
	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,1);

	auto pose_tf = Eigen::make_tf(pose);

	// convert msg to correst geometry_msgs::PoseStamped type
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = geometry_msgs::make_pose(pose_tf);
	pose_stamped.header.stamp.sec = ros::Time().sec;

	// send message
	pub_pose.publish(pose_stamped);
}

void
teleop_grasp::command_gripper(const teleop_grasp::GripperState& open_or_close)
{
	// ROS_WARN_STREAM("sending command to gripper...");

	actionlib::SimpleActionClient<franka_gripper::MoveAction> action("/franka_gripper/move",true);
	action.waitForServer();

	// ROS_WARN_STREAM("sending action to gripper...");

	franka_gripper::MoveAction msg;
	msg.action_goal.goal.speed = 0.1;

	// ROS_WARN_STREAM("OPEN? " << bool(open_or_close));

	if (bool(open_or_close) == teleop_grasp::gesture_state_prev )
		return;

	if (open_or_close == teleop_grasp::GripperState::OPEN)
	{
		teleop_grasp::gesture_state_prev = true;
		msg.action_goal.goal.width = 0.045;
		action.sendGoal(msg.action_goal.goal);
	}
	else
	{
		// ROS_WARN_STREAM("CLOSING...");
		teleop_grasp::gesture_state_prev = false;
		msg.action_goal.goal.width = 0.01;
		action.sendGoal(msg.action_goal.goal);
	}
}

geometry_msgs::Pose
teleop_grasp::predictor::predict_pose_linear()
{
	// compute diff transform from prev hand pose to current hand pose
	auto d_pose = Eigen::make_tf(teleop_grasp::pose_hand_prev).inverse() * Eigen::make_tf( teleop_grasp::pose_hand );

	// extend relative prediction vector
	d_pose.translation() *= 2;

	// compute new desired ee pose
	teleop_grasp::pose_ee_des = geometry_msgs::make_pose( Eigen::make_tf( teleop_grasp::pose_ee ) * d_pose );

	// convert to geometry_msgs::Pose and return
	return teleop_grasp::pose_ee_des;
}

geometry_msgs::Pose
teleop_grasp::utils::get_current_hand_pose()
{
	static ros::NodeHandle nh;
	auto topic_pose_hand = ros::param::param<std::string>("/teleoperation/topic_pose_hand","");
	return *(ros::topic::waitForMessage<geometry_msgs::Pose>(topic_pose_hand,nh));
}

geometry_msgs::Pose
teleop_grasp::utils::get_current_franka_pose()
{
	static ros::NodeHandle nh;
	auto franka_state     = *(ros::topic::waitForMessage<franka_msgs::FrankaState>("/franka_state_controller/franka_states",nh));
	return geometry_msgs::make_pose(Eigen::Isometry3d(Eigen::Matrix4d::Map(franka_state.O_T_EE.data())));
}

void
teleop_grasp::utils::set_current_franka_pose(const geometry_msgs::Pose &des_pose)
{
	static ros::NodeHandle nh;
	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_pd_nullspace_controller/command",1);

	// convert msg to correst geometry_msgs::PoseStamped type
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = des_pose;
	pose_stamped.header.stamp.sec = ros::Time().sec;

	// send message
	pub_pose.publish(pose_stamped);
}

bool
teleop_grasp::utils::has_any_change_occurred(const Eigen::Isometry3d& pose1_tf,const Eigen::Isometry3d& pose2_tf )
{
	auto d_pose_tf = pose1_tf.inverse() * pose2_tf;
	if ( d_pose_tf.translation().squaredNorm() < teleop_grasp::CHANGE_THRESHOLD and Eigen::AngleAxisd(d_pose_tf.rotation() ).angle() < teleop_grasp::CHANGE_THRESHOLD )
		return false;
	else
		return true;
}

// bool 
// teleop_grasp::utils::has_too_much_change_occurred(const geometry_msgs::Pose& pose)
// {
// 	ROS_WARN_STREAM(" pose bad?! " << pose);
// 	if (pose.position.x > 1.2 or pose.position.x < -1.2 or pose.position.y > 1.2 or pose.position.y < -1.2 or pose.position.z > 1.5 or pose.position.z < 0.2)
// 		return true;
// 	else
// 		return false;
// }

bool 
teleop_grasp::utils::has_too_much_change_occurred(const Eigen::Isometry3d& pose1_tf,const Eigen::Isometry3d& pose2_tf)
{
	auto d_pose_tf = pose1_tf.inverse() * pose2_tf;
	if (d_pose_tf.translation().squaredNorm() > teleop_grasp::MAX_LIN_VELOCITY and Eigen::AngleAxisd(d_pose_tf.rotation() ).angle() > teleop_grasp::MAX_ANG_VELOCITY)
		return true;
	else
		return false;
}

geometry_msgs::Pose
teleop_grasp::utils::restrict_pose(geometry_msgs::Pose pose)
{

	// if ( has_too_much_change_occurred(pose) )
	// {
	// 	ROS_WARN_STREAM("hey i cannot move");
	// 	return teleop_grasp::calibrate_pose_franka;
	// }

	pose.position.x = pose.position.x * 2 - 1.0;
	pose.position.y = pose.position.y * 2 - 1.0;

	pose.position.x = pose.position.x + 0.5;
	pose.position.z = pose.position.z + 0.2;

	pose.position.y *= -1.0;

	return pose;
}