#pragma once
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <teleop_grasp/teleop_grasp.h>

class Teleoperation
{
public:
	Teleoperation(ros::NodeHandle& nh);
	~Teleoperation();

	void 
	test_system();

	bool 
	init();

private:
	
	std::string gasp_topic = "grasp";
	std::string pose_topic = "pose";


	ros::Subscriber gesture_est;
	ros::Subscriber pose_est;
	ros::Publisher gasp;
	ros::Publisher pose;

	ros::NodeHandle nh;

};

