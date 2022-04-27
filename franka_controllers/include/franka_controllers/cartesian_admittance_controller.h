#pragma once

#include <vector>
#include <mutex>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace Eigen
{
	using Vector6d = Eigen::Matrix<double,6,1>;
	using Matrix6d = Eigen::Matrix<double,6,6>;
	using Matrix6x7d = Eigen::Matrix<double,6,7>;
	using Vector7d = Eigen::Matrix<double,7,1>;
	using Matrix7d = Eigen::Matrix<double,7,7>;
}

namespace franka_controllers
{
	class CartesianAdmittanceController final : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
	                                                                                                  hardware_interface::EffortJointInterface,
	                                                                                                  franka_hw::FrankaStateInterface>
	{
	public:

		static inline constexpr auto CONTROLLER_NAME = "CartesianPDController";

		CartesianAdmittanceController() {}
		~CartesianAdmittanceController() { sub_command.shutdown(); }

		bool
		init(hardware_interface::RobotHW *hw, ros::NodeHandle& nh) override;

		void
		starting(const ros::Time& time) override;

		void
		update(const ros::Time& time, const ros::Duration& period) override;

	private:

		std::string                                   arm_id;
		size_t                                        num_joints;
		std::vector<std::string>                      joint_names;
		std::vector<hardware_interface::JointHandle>  joint_handles;
		std::unique_ptr<franka_hw::FrankaModelHandle> model_handle;
		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;

		std::mutex                                    mtx_T_ref;
		Eigen::Isometry3d                             T_d, T_ref;
		Eigen::Vector7d                               qN_d;

		Eigen::Matrix6d                               kp, kd;
		Eigen::Matrix3d                               Kp, Ko, Dp, Do, Mp, Mo;
		double                                        kpp, kpo, kvp, kvo, kn, kc;
		double                                        dtau_max;
		double                                        slew_rate;

		ros::Subscriber                               sub_command;

		struct RobotState
		{
			Eigen::Isometry3d T_e;
			Eigen::Vector7d q;
			Eigen::Vector7d dq;
			Eigen::Vector6d h_e;
		};

		struct RobotDynamics
		{
			Eigen::Matrix6x7d J;
			Eigen::Matrix6x7d dJ;
			Eigen::Matrix7d M;
			Eigen::Vector7d C;
			Eigen::Vector7d g;
		};

		std::tuple<Eigen::Vector6d, Eigen::Vector6d, Eigen::Vector6d>
		get_desired(const Eigen::Isometry3d& T_ref);

		RobotState
		get_robot_state();

		RobotDynamics
		get_robot_dynamics(const Eigen::Vector7d& q, const Eigen::Vector7d& dq, double dt);
		
		std::tuple<Eigen::Vector6d, Eigen::Vector6d, Eigen::Vector6d>
		spatial_impedance(const Eigen::Vector6d& x_d, const Eigen::Vector6d& dx_d, const Eigen::Vector6d& ddx_d, const Eigen::Vector6d& h_e, double dt);

		Eigen::Vector6d
		get_pose_error(const Eigen::Isometry3d& T_e, const Eigen::Isometry3d& T_d);

		Eigen::Isometry3d
		filter_desired_pose(const Eigen::Isometry3d& pose_ref);

		Eigen::Vector7d
		saturate_rotatum(const Eigen::Vector7d& tau_des, const double period = 0.001 /* [s] */);

		void
		callback_command(const geometry_msgs::PoseStampedConstPtr& msg);

	};
}