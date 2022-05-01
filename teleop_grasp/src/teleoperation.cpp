#include "geometry_msgs/Pose.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "teleop_grasp/teleop_grasp.h"
#include <cstddef>
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

	ROS_WARN_STREAM("topic_grasp_state: " << topic_grasp_state);
	ROS_WARN_STREAM("topic_pose_hand: " << topic_pose_hand);
	ROS_WARN_STREAM("topic_franka_pose_ee: " << topic_franka_pose_ee);

	// -- Synthetic data points ---------------------------------------
	// set a non-centered hand pose
	// teleop_grasp::pose_hand = teleop_grasp::pose_ee;
	// teleop_grasp::pose_hand.position.x += 10;
	// teleop_grasp::pose_hand.position.y += 10;
	// teleop_grasp::pose_hand.position.z += 10;

	// std::vector<geometry_msgs::Pose> v_poses;
	// auto p1 = teleop_grasp::pose_hand; p1.position.y += 0.1;
	// auto p2 = teleop_grasp::pose_hand; p2.position.z += 0.1;
	// auto p3 = teleop_grasp::pose_hand; p3.position.y += -0.1;
	// auto p4 = teleop_grasp::pose_hand; p4.position.z += -0.1;
	// v_poses = { p1,p2,p3,p4 };

	teleop_grasp::calibrate();

	ros::Rate rate(100);

	while (ros::ok()) 
	{
		std::cout << "running.................................................." << std::endl;

		// open and close gripper
		auto gripper_state = (teleop_grasp::gesture_state) ? teleop_grasp::GripperState::OPEN : teleop_grasp::GripperState::CLOSE;
		teleop_grasp::command_gripper( gripper_state );

		// predict the next position of hand
		auto pose_ee_pred = teleop_grasp::predictor::predict_pose_linear();

		// compute the new position based on relative difference between hand poses
		teleop_grasp::pose_ee_des = teleop_grasp::compute_desired_ee_pose( teleop_grasp::pose_hand );

		// send pose to franka | choose between predicted pose, and desired pose
		teleop_grasp::command_pose_robot( teleop_grasp::pose_ee_des, topic_franka_pose_ee );


		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}