#pragma once

#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <std_msgs/Float64MultiArray.h>

namespace Eigen
{
	using Vector7d  = Eigen::Matrix<double,7,1>;
	using Matrix7d  = Eigen::Matrix<double,7,7>;
}

namespace franka_controllers
{
	class JointPositionPDGravityController final : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
	                                                                                                     hardware_interface::EffortJointInterface,
	                                                                                                     franka_hw::FrankaStateInterface>
	{
	public:
	
		static inline constexpr auto CONTROLLER_NAME = "JointPositionPDGravityController";

		JointPositionPDGravityController() {}
		~JointPositionPDGravityController() { sub_command.shutdown(); }

		bool 
		init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh) override;

		void 
		starting(const ros::Time &time) override;

		void 
		update(const ros::Time &time, const ros::Duration &period) override;

	private:

		std::string                                         arm_id;
		size_t                                              num_joints;
		std::vector<std::string>                            joint_names;
		std::vector<hardware_interface::JointHandle>        joint_handles;
		std::unique_ptr<franka_hw::FrankaModelHandle>       model_handle;
		std::unique_ptr<franka_hw::FrankaStateHandle>       state_handle;
		
		Eigen::Matrix7d                                     Kp; // stiffness
		Eigen::Matrix7d                                     Kd; // dampening
		double                                              dtau_max;
		
		ros::Subscriber                                     sub_command;
		realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
		Eigen::Vector7d                                     q_d;

		Eigen::Vector7d
		get_position();

		Eigen::Vector7d
		get_velocity();

		Eigen::Vector7d
		get_gravity();

		Eigen::Vector7d
		saturate_rotatum(const Eigen::Vector7d &tau_des, const double period = 0.001 /* [s] */);

		void
		callback_command(const std_msgs::Float64MultiArrayConstPtr &msg);
	};
}