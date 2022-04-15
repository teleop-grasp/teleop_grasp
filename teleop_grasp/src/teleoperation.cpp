
#include "geometry_msgs/Pose.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <teleoperation/teleoperation.h>



/*
todo:
 + init -> look for topics, do they exist?
 + give callbacks to subs
*/



Teleoperation::Teleoperation(ros::NodeHandle& nh) 
{
	this->nh = nh;
}

Teleoperation::~Teleoperation() 
{

}

void 
Teleoperation::test_system()
{

}

bool 
Teleoperation::init()
{

	// sub 
	gesture_est = nh.subscribe<std_msgs::Bool>( "gesture_est", 1, &teleop_grasp::gesture_est);
	pose_est = nh.subscribe<geometry_msgs::Pose>( "pose_est", 1, &teleop_grasp::pose_est);

	// pub
	gasp = nh.advertise<std_msgs::Bool>(this->gasp_topic, 1000);
	pose = nh.advertise<std_msgs::Float64>(this->pose_topic,1000);

	return false;
}



int main(int argc, char** argv) 
{

	ros::init(argc, argv, "teleoperation");

	ros::NodeHandle nh;

	Teleoperation tel(nh);

	tel.init();

	std::cout << "test" << std::endl;

	return 0;
}