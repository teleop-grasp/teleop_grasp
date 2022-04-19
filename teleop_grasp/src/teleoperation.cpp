
#include "geometry_msgs/Pose.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "teleop_grasp/teleop_grasp.h"

namespace teleoperation
{
	std::string grasp = "";
	std::string pose = "";
	std::string name = "teleoperation";

	ros::Subscriber grasp_sub;
	ros::Subscriber pose_sub;

	void gesture_est(const std_msgs::BoolConstPtr& msg)
	{
		auto msg_bool = bool(msg->data);
		if ( msg_bool )
			teleop_grasp::set_gripper(teleop_grasp::CMD::OPEN);
		else
			teleop_grasp::set_gripper(teleop_grasp::CMD::CLOSE);
	}

	void follow_franka(const geometry_msgs::PoseConstPtr& msg)
	{
		ROS_INFO("[FOLLOW_FRANKA]");
	}


	bool init(ros::NodeHandle& nh)
	{
		if (not nh.getParam("teleop_grasp/grasp_topic", grasp)) 
		{
			ROS_ERROR_STREAM(name << ": Could not read parameter grasp");
			return false;
		}

		if (not nh.getParam("teleop_grasp/pose_topic", pose))
		{
			ROS_ERROR_STREAM(name << ": Could not read parameter pose");
			return false;
		}

		grasp_sub = nh.subscribe<std_msgs::Bool>(grasp, 1,&teleoperation::gesture_est);
		pose_sub = nh.subscribe<geometry_msgs::Pose>(pose,1,&teleoperation::follow_franka);

		return true;
	}
}

int 
main(int argc, char** argv) 
{
	ros::init(argc, argv, teleoperation::name);
	ros::NodeHandle nh;
	teleoperation::init(nh);

	ROS_INFO_STREAM("starting " << nh.getNamespace() << "..." );

	ros::Rate rate(0.5);

	while (ros::ok()) 
		ros::spinOnce();

	return 0;
}