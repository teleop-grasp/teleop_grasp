#include <ros/ros.h>
#include <teleop_grasp/teleop_grasp.h>

int
main(int argc, char **argv)
{
	// usage:
	// roslaunch teleop_grasp pipeline.launch

	// ------------------------------------------------------------------------------

	// init ROS node
	ros::init(argc, argv, "teleop_grasp");
	ros::NodeHandle nh;

	// ------------------------------------------------------------------------------

	// .yaml files holds configuration of system
	// such as cv_camera topic, loaded in teleop_grasp::init()
	// teleop_grasp::get_img() etc. are wrappers of ROS interface to get images etc.
	
	// setup system
	if (not teleop_grasp::init())
		ROS_BREAK(); // or throw std::exception

	// main loop
	ros::Rate rate(100); // Hz
	while (ros::ok())
	{
		// const auto img = get_img();

		// const bool gesture = get_gesture(img);
		// const auto arm_pose = get_arm_pose(img);

		// const auto pose_ee_d = get_desired_ee_pose(arm_pose);

		// const auto [q, dq] = get_state();
		// const auto pose_ee = get_pose_ee();
		// const auto traj = get_traj(pose_ee, pose_ee_d);
		// const auto traj_opt = optimize_traj(traj);
		
		// franka_controllers::command_traj(traj_opt);

		rate.sleep();
	}

	// exit
	return 0;
}