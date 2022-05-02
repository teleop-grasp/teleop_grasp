#include "franka_gripper/MoveAction.h"
#include "franka_msgs/FrankaState.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros_utils/eigen.h"
#include <teleop_grasp/teleop_grasp.h>

void 
teleop_grasp::calibrate()
{

	static ros::NodeHandle nh;

	// get robot ee pose
	ROS_WARN_STREAM("testing...");
	auto franka_state     = *(ros::topic::waitForMessage<franka_msgs::FrankaState>("/franka_state_controller/franka_states",nh));
	teleop_grasp::pose_ee = geometry_msgs::make_pose(Eigen::Isometry3d(Eigen::Matrix4d::Map(franka_state.O_T_EE.data())));

	// get hand pose
	auto topic_pose_hand = ros::param::param<std::string>("/teleoperation/topic_pose_hand","");
	teleop_grasp::pose_hand = *(ros::topic::waitForMessage<geometry_msgs::Pose>(topic_pose_hand,nh));

	// update prev hand pose
	teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand;

	// get current position of robot ee and orientation of hand, then send to robot
	auto t = Eigen::make_tf(pose_ee).translation();
	auto R = Eigen::make_tf(teleop_grasp::pose_hand).rotation();

	Eigen::Matrix4d cali_pose;
	cali_pose.setIdentity();
	cali_pose.block<3,3>(0,0) = R;
	cali_pose.block<3,1>(0,3) = t;

	command_pose_robot( geometry_msgs::make_pose( Eigen::Isometry3d(cali_pose) ) );

	// calibration completed
	ROS_INFO_STREAM("calibration has been completed with:\n\trobot init tcp pose: " << teleop_grasp::pose_ee << "\n\thand init pose: " << teleop_grasp::pose_hand);
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

	// compute diff transform from prev hand pose to current hand pose
	auto d_pose_tf = pose_hand_prev_tf.inverse() * pose_hand_tf;

	// if the velocity becomes too large
	if (  d_pose_tf.translation().squaredNorm() > teleop_grasp::MAX_LIN_VELOCITY and Eigen::AngleAxisd(d_pose_tf.rotation() ).angle() > teleop_grasp::MAX_ANG_VELOCITY )
	{
		ROS_WARN_STREAM("velocity too large...");
		d_pose_tf = Eigen::Isometry3d::Identity();
	}

	// if no changes are made to the pose, this is caused by the hand moving outside the camera's view
	if (  d_pose_tf.translation().squaredNorm() < teleop_grasp::CHANGE_THRESHOLD and Eigen::AngleAxisd(d_pose_tf.rotation() ).angle() < teleop_grasp::CHANGE_THRESHOLD)
	{

		ROS_WARN_STREAM("no changes are made...");
		d_pose_tf = Eigen::Isometry3d::Identity();
	}

	teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand;

	// compute new desired ee pose
	teleop_grasp::pose_ee_des = geometry_msgs::make_pose( Eigen::make_tf( teleop_grasp::pose_ee ) * d_pose_tf );

	// convert back to geometry_msgs::Pose and return
	return teleop_grasp::pose_ee_des;
}

void
teleop_grasp::command_pose_robot(const geometry_msgs::Pose& pose, const std::string& topic_pose )
{
	static ros::NodeHandle nh;
	static ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,1);

	ROS_WARN_STREAM("topic_pose: " << topic_pose);
	ROS_WARN_STREAM("des_pose.pos: " << pose.position);
	ROS_WARN_STREAM("commanding a pose to the robot...\n" << pose.position);

	// convert msg to correst geometry_msgs::PoseStamped type
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = pose;
	pose_stamped.header.stamp.sec = ros::Time().sec;

	// send message
	pub_pose.publish(pose_stamped);
}

void
teleop_grasp::command_gripper(const teleop_grasp::GripperState& open_or_close)
{
	ROS_WARN_STREAM("sending command to gripper...");

	actionlib::SimpleActionClient<franka_gripper::MoveAction> action("/franka_gripper/move",true);
	
	action.waitForServer();

	ROS_WARN_STREAM("sending action to gripper...");

	franka_gripper::MoveAction msg;
	msg.action_goal.goal.speed = 0.1;

	ROS_WARN_STREAM((open_or_close == teleop_grasp::GripperState::OPEN));

	if (open_or_close == teleop_grasp::GripperState::OPEN)
	{
		teleop_grasp::gesture_state = true;
		msg.action_goal.goal.width = 0.045;
		action.sendGoal(msg.action_goal.goal);
	}
	else
	{
		teleop_grasp::gesture_state = false;
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