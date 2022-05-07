#pragma once

#include <vector>
#include <mutex>
#include <thread>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <realtime_tools/realtime_buffer.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_controllers/CartesianAdmittanceControllerDebug.h>

namespace Eigen
{
	using Vector6d = Eigen::Matrix<double,6,1>;
	using Matrix6d = Eigen::Matrix<double,6,6>;
	using Matrix6x7d = Eigen::Matrix<double,6,7>;
	using Matrix7x6d = Eigen::Matrix<double,7,6>;
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

		static inline constexpr auto CONTROLLER_NAME = "CartesianAdmittanceController";

		CartesianAdmittanceController() {}
		~CartesianAdmittanceController() { sub_command.shutdown(); }

		bool
		init(hardware_interface::RobotHW *hw, ros::NodeHandle& nh) override;

		void
		starting(const ros::Time& time) override;

		void
		update(const ros::Time& time, const ros::Duration& period) override;

	private:

		using Pose = std::tuple<Eigen::Vector3d, Eigen::Matrix3d>; // (p, R)
		using Frame = std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>; // (p, R, dp, w, ddp, dw)

		struct RobotState
		{
			Eigen::Vector3d p_e;
			Eigen::Matrix3d R_e;
			Eigen::Vector7d q;
			Eigen::Vector7d dq;
			Eigen::Vector6d h_e;
		};

		struct RobotDynamics
		{
			Eigen::Matrix6x7d J;
			Eigen::Matrix6x7d dJ;
			Eigen::Matrix7d M;
			Eigen::Vector7d c; // c = C*dq
			Eigen::Vector7d g;
		};

		std::string                                   arm_id;
		size_t                                        num_joints;
		std::vector<std::string>                      joint_names;
		std::vector<hardware_interface::JointHandle>  joint_handles;
		std::unique_ptr<franka_hw::FrankaModelHandle> model_handle;
		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;

		realtime_tools::RealtimeBuffer<Pose>          buffer_pose_ref;
		Pose                                          pose_d; // CHANGE TO STRUCT
		Eigen::Vector7d                               qN_d;

		// Eigen::Matrix6d                               kp, kd;
		Eigen::Matrix3d                               Kp, Ko, Dp, Do, Mp, Mo;
		double                                        kpp, kpo, kvp, kvo, kn, kc;
		double                                        dtau_max;
		double                                        slew_rate;
		bool                                          in_simulation;
		double                                        state_publish_rate;

		ros::Subscriber                               sub_command;

		std::mutex                                    mtx_msg_debug;
		ros::Publisher                                pub_msg_debug;
		ros::Publisher                                pub_msg_T_c;
		std::thread                                   thread_msg_debug;
		CartesianAdmittanceControllerDebug            msg_debug;

		Frame
		get_desired(const Eigen::Vector3d& p_ref, const Eigen::Matrix3d& R_ref);

		RobotState
		get_robot_state();

		RobotDynamics
		get_robot_dynamics(const Eigen::Vector7d& q, const Eigen::Vector7d& dq, double dt = 0.001);

		Frame
		spatial_impedance(
			const Eigen::Vector3d& p_d,
			const Eigen::Matrix3d& R_d,
			const Eigen::Vector3d& dp_d,
			const Eigen::Vector3d& w_d,
			const Eigen::Vector3d& ddp_d,
			const Eigen::Vector3d& dw_d,
			const Eigen::Vector6d& h_e,
			double dt = 0.001
		);

		Eigen::Vector6d
		pos_ori_control(
			const Eigen::Vector3d& p_c,
			const Eigen::Vector3d& p_e,
			const Eigen::Matrix3d& R_c,
			const Eigen::Matrix3d& R_e,
			const Eigen::Vector3d& dp_c,
			const Eigen::Vector3d& dp_e,
			const Eigen::Vector3d& w_c,
			const Eigen::Vector3d& w_e,
			const Eigen::Vector3d& ddp_c,
			const Eigen::Vector3d& dw_c
		);

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