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
	auto franka_state     = *(ros::topic::waitForMessage<franka_msgs::FrankaState>("/franka_state_controller/franka_states",nh));
	teleop_grasp::pose_ee = geometry_msgs::make_pose(Eigen::Isometry3d(Eigen::Matrix4d::Map(franka_state.O_T_EE.data())));

	// get hand pose
	auto topic_pose_hand = ros::param::param<std::string>("/teleoperation/topic_pose_hand","");
	teleop_grasp::pose_hand = *(ros::topic::waitForMessage<geometry_msgs::Pose>(topic_pose_hand,nh));

	// x-axis offset
	teleop_grasp::pose_hand.position.x += 0.2;

	// update prev hand pose
	teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand;

	// get current position of robot ee and orientation of hand, then send to robot

	auto t = Eigen::make_tf(teleop_grasp::pose_ee).translation();
	auto R = Eigen::make_tf(teleop_grasp::pose_hand).rotation();

	// t.z() += 0.5;
	// t.x() += 0.5;

	Eigen::Isometry3d cali_pose;
	cali_pose.setIdentity();
	cali_pose.rotate(R).translate(t);

	// rotate to same frame as hand
	// cali_pose = utils::rotate_to_hand_frame(Eigen::Isometry3d(cali_pose));

	ROS_WARN_STREAM("calibrated position: \n" << cali_pose.matrix());

	command_pose_robot( geometry_msgs::make_pose( cali_pose ) );

	// calibration completed
	ROS_WARN_STREAM("calibration has been completed with:\n\trobot init tcp pose: " << teleop_grasp::pose_ee << "\n\thand init pose: " << teleop_grasp::pose_hand);
}

std::string
teleop_grasp::get_topic(const std::string& topic)
{
	std::string topic_out = "";

	if (not ros::param::get("~" + topic,topic_out)) 
	{
		ROS_ERROR_STREAM("Could not read parameter " << topic);
		return topic_out;
	}
	return topic_out;
}

geometry_msgs::Pose
teleop_grasp::compute_desired_ee_pose(const geometry_msgs::Pose& pose_hand)
{
	// convert to transformation matrix
	auto pose_hand_tf      = Eigen::make_tf(teleop_grasp::pose_hand);
	auto pose_hand_prev_tf = Eigen::make_tf(teleop_grasp::pose_hand_prev);
	// auto pose_hand_tf      = Eigen::make_tf(teleop_grasp::pose_hand) * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX());
	// auto pose_hand_prev_tf = Eigen::make_tf(teleop_grasp::pose_hand_prev) * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX());

	// expand interval
	// pose_hand_tf.translation().x()      = pose_hand_tf.translation().x() * 2 - 1.0;      
	// pose_hand_prev_tf.translation().x() = pose_hand_prev_tf.translation().x() * 2 - 1.0; 

	// compute diff transform from prev hand pose to current hand pose
	auto d_pose_tf = pose_hand_prev_tf.inverse() * pose_hand_tf;

	// if the velocity becomes too large
	if ( utils::has_too_much_change_occurred(pose_hand_prev_tf, pose_hand_tf)  )
	{
		ROS_WARN_STREAM("velocity too large...");
		d_pose_tf = Eigen::Isometry3d::Identity();
		return geometry_msgs::make_pose( pose_hand_tf );
	}

	// if no changes are made to the pose, this is caused by the hand moving outside the camera's view
	if ( utils::has_any_change_occurred(pose_hand_prev_tf, pose_hand_tf) )
	{
		ROS_WARN_STREAM("no changes are made...");
		d_pose_tf = Eigen::Isometry3d::Identity();
		return geometry_msgs::make_pose( pose_hand_tf );
	}

	teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand;

	ROS_WARN_STREAM("d_pose_tf = \n" << d_pose_tf.matrix());

	// compute new desired ee pose
	teleop_grasp::pose_ee_des = geometry_msgs::make_pose( Eigen::make_tf( teleop_grasp::pose_ee ) * d_pose_tf );
	teleop_grasp::pose_ee = teleop_grasp::pose_ee_des;

	return teleop_grasp::pose_ee_des;
}

void
teleop_grasp::command_pose_robot(const geometry_msgs::Pose& pose, const std::string& topic_pose )
{
	static ros::NodeHandle nh;
	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,1);

	// ROS_WARN_STREAM("des_pose.pos: \n" << pose.position);
	// ROS_WARN_STREAM("commanding a pose to the robot...\n" << pose.position);


	ROS_WARN_STREAM("I WANT THIS POSE PLZ... POSE\n" << pose);

	auto pose_tf = Eigen::make_tf(pose);
	
	// pose_tf = utils::rotate_to_hand_frame(pose_tf);

	// pose_tf.translation().z() += 0.5;
	// pose_tf.translation().x() += 0.5;

	// if (pose_tf.translation().z() < 0.5)
	// {
	// 	ROS_WARN_STREAM("WARN");
	// 	pose_tf.translation().z() = 0.5;
	// }

	ROS_WARN_STREAM("I WANT THIS POSE PLZ...\n" << pose_tf.matrix());

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
	{
		// ROS_WARN("same...");
		return;
	}

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

Eigen::Isometry3d
teleop_grasp::utils::rotate_to_hand_frame(Eigen::Isometry3d tf)
{
	// rotate ee of franka to fit world frame
	// rotate π/2 about y 
	auto R1 = Eigen::Matrix3d(Eigen::AngleAxisd( M_PI / 2.0, Eigen::Vector3d::UnitY()));
	// rotate -π/2 about z
	auto R2 = Eigen::Matrix3d(Eigen::AngleAxisd( -M_PI / 2.0, Eigen::Vector3d::UnitZ()));

	return tf.rotate(R1).rotate(R2);
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

bool 
teleop_grasp::utils::has_too_much_change_occurred(const Eigen::Isometry3d& pose1_tf,const Eigen::Isometry3d& pose2_tf)
{
	auto d_pose_tf = pose1_tf.inverse() * pose2_tf;
	if (d_pose_tf.translation().squaredNorm() > teleop_grasp::MAX_LIN_VELOCITY and Eigen::AngleAxisd(d_pose_tf.rotation() ).angle() > teleop_grasp::MAX_ANG_VELOCITY)
		return true;
	else
		return false;
}