
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "teleop_grasp/teleop_grasp.h"
#include <cstddef>
#include <vector>

int 
main(int argc, char** argv) 
{
	ros::init(argc, argv, "teleoperation");
	ros::NodeHandle nh;

	ros::Subscriber sub_grasp = nh.subscribe<std_msgs::Bool>(teleop_grasp::get_grasp_topic(),1, [&](const auto& msg){ teleop_grasp::gesture_state = (*msg).data;});
	ros::Subscriber sub_pose  = nh.subscribe<geometry_msgs::Pose>(teleop_grasp::get_pose_topic(),1,[&](const auto& msg){teleop_grasp::pose_hand_prev = teleop_grasp::pose_hand; teleop_grasp::pose_hand = *msg;}); 

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
	// v_poses = {p1,p2,p3,p4};

	// std::cin.get();
	teleop_grasp::calibrate();

	ros::Rate rate(0.5);

	while (ros::ok()) 
	{
		auto gripper_state = (teleop_grasp::gesture_state) ? teleop_grasp::GripperState::OPEN : teleop_grasp::GripperState::CLOSE;
		teleop_grasp::set_gripper(gripper_state);

		teleop_grasp::pose_ee_des = teleop_grasp::compute_desired_ee_pose( teleop_grasp::pose_hand );
		teleop_grasp::set_pose_robot(teleop_grasp::pose_ee_des);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}