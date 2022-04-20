#include <franka_controllers/cartesian_pd_nullspace_controller.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros_utils/eigen.h>

// export controller
PLUGINLIB_EXPORT_CLASS(franka_controllers::CartesianPDNullspaceController, controller_interface::ControllerBase)

namespace franka_controllers
{

bool
CartesianPDNullspaceController::init(hardware_interface::RobotHW *hw, ros::NodeHandle& nh)
{

	// read arm_id from config file
	if (not nh.getParam("arm_id", arm_id))
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read parameter arm_id");
		return false;
	}

	// read joint_names from config file
	if (not nh.getParam("joint_names", joint_names) || joint_names.size() != 7)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Invalid or no joint_names parameters provided");
		return false;
	}

	// determine number of joints
	if (num_joints = joint_names.size(); num_joints != 7)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Invalid number of joints; got " << num_joints << ", expected " << 7);
		return false;
	}

	// load model interface
	auto* model_interface = hw->get<franka_hw::FrankaModelInterface>();
	if (not model_interface)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Error getting model interface from hardware");
		return false;
	}

	// load model handle
	try
	{
		model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Exception getting model handle from interface: " << ex.what());
		return false;
	}

	// load state interface
	auto* state_interface = hw->get<franka_hw::FrankaStateInterface>();
	if (not state_interface)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Error getting state interface from hardware");
		return false;
	}

	// load state handle
	try
	{
		state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Exception getting state handle from interface: " << ex.what());
		return false;
	}

	// load effort joint interface
	auto* effort_joint_interface = hw->get<hardware_interface::EffortJointInterface>();
	if (not effort_joint_interface)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Error getting effort joint interface from hardware");
		return false;
	}

	// load joint handles
	for (const auto& joint_name : joint_names)
	{
		try
		{
			joint_handles.push_back(effort_joint_interface->getHandle(joint_name));
		}
		catch (const hardware_interface::HardwareInterfaceException& ex)
		{
			ROS_ERROR_STREAM(CONTROLLER_NAME << ": Exception getting joint handles: " << ex.what());
			return false;
		}
	}

	// read kp gains from config
	if (std::vector<double> vec_kp; nh.getParam("kp", vec_kp) and vec_kp.size() == 6)
	{
		kp = Eigen::Vector6d(vec_kp.data()).asDiagonal();
	}
	else
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read kp gain");
		return false;
	}

	// read kd gains from config
	if (std::vector<double> vec_kd; nh.getParam("kd", vec_kd) and vec_kd.size() == 6)
	{
		kd = Eigen::Vector6d(vec_kd.data()).asDiagonal();
	}
	else
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read kd gain");
		return false;
	}

	// read kn gains from config
	if (not nh.getParam("kn", kn))
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read kn gain");
		return false;
	}

	// read dtau_max from config (-1 to disable)
	if (not nh.getParam("dtau_max", dtau_max))
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read dtau_max");
		return false;
	}

	// read slew_rate from config
	if (not nh.getParam("slew_rate", slew_rate))
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read slew_rate");
		return false;
	}

	// subscribe to command topic for desired pose
	sub_command = nh.subscribe<geometry_msgs::PoseStamped>(
		"command", // topic name
		20, // queue size
		&CartesianPDNullspaceController::callback_command, // function pointer
		this,
		ros::TransportHints().reliable().tcpNoDelay() // transport hints
	);

	// init complete
	ROS_WARN_STREAM("Initialized " << CONTROLLER_NAME << " with:\n\n"
		<< "kp = "        << kp.diagonal().transpose() << "\n"
		<< "kd = "        << kd.diagonal().transpose() << "\n"
		<< "kn = "        << kn << "\n"
		<< "dtau_max = "  << dtau_max << "\n"
		<< "slew_rate = " << slew_rate  << "\n"
	);

	return true;
}

void
CartesianPDNullspaceController::starting(const ros::Time& time)
{
	// set equilibrium point to current state
	T_d = T_ref = get_robot_state().T_e;

	// set desired nullspace to initial robot state
	qN_d = get_robot_state().q;
}

void
CartesianPDNullspaceController::update(const ros::Time& time, const ros::Duration& period)
{
	// elapsed time
	static ros::Duration elapsed_time = ros::Duration(0.);
	elapsed_time += period;

	// read robot state and dynamics
	const auto [T_e, q, dq] = get_robot_state();
	const auto [J, M, C, g] = get_robot_dynamics();
	const auto dx_e = J * dq;

	// compute errors (in base frame)
	const auto x_de = get_pose_error(T_e, T_d); // [p_de, eps_de]
	const auto dx_de = Eigen::Vector6d::Zero() - dx_e; // no desired velocity

	// cartesian torque
	const auto tau_task = J.transpose() * (kp * x_de + kd * dx_de);

	// const auto M_xe = (J * M.inverse() * J.transpose()).inverse();
	// Eigen::Vector7d u = J.transpose() * M_xe * (kp * x_de + kd * dx_de) + g;

	// nullspace torque
	// https://studywolf.wordpress.com/2013/09/17/robot-control-5-controlling-in-the-null-space/
	const auto J_T_pinv = Eigen::pseudo_inverse(J.transpose(), 0.2); // damped pseudo inverse
	const auto I7x7d = Eigen::Matrix7d::Identity();
	const auto kc = 2.0 * sqrt(kn); // damping ratio of 1.0
	const auto tau_null = (I7x7d - J.transpose() * J_T_pinv) * (kn * (qN_d - q) - kc * dq);

	// desired joint torque
	Eigen::Vector7d tau_d = tau_task + tau_null + C;

	// saturate rate-of-effort (rotatum)
	if (dtau_max > 0)
		tau_d = saturate_rotatum(tau_d, period.toSec());

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		joint_handles[i].setCommand(tau_d[i]);

	// apply slew rate (filter) on reference pose
	T_d = filter_desired_pose(T_ref);
}

CartesianPDNullspaceController::RobotState
CartesianPDNullspaceController::get_robot_state()
{
	const auto robot_state = state_handle->getRobotState();
	return
	{
		Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())),
		Eigen::Vector7d(robot_state.q.data()),
		Eigen::Vector7d(robot_state.dq.data())
	};
}


CartesianPDNullspaceController::RobotDynamics
CartesianPDNullspaceController::get_robot_dynamics()
{
	return
	{
		Eigen::Matrix6x7d::Map(model_handle->getZeroJacobian(franka::Frame::kEndEffector).data()),
		Eigen::Matrix7d::Map(model_handle->getMass().data()),
		Eigen::Vector7d(model_handle->getCoriolis().data()),
		Eigen::Vector7d(model_handle->getGravity().data())
	};
}

Eigen::Vector7d
CartesianPDNullspaceController::saturate_rotatum(const Eigen::Vector7d& tau_d, const double dt)
{
	// previous desired torque and saturated torque
	static Eigen::Vector7d tau_d_prev = Eigen::Vector7d::Zero();
	static Eigen::Vector7d tau_d_sat = Eigen::Vector7d::Zero();

	// compute saturated torque
	for (size_t i = 0; i < tau_d_sat.size(); ++i)
	{
		const double dtau = (tau_d[i] - tau_d_prev[i]) / dt; // dtau/dt
		tau_d_sat[i] = tau_d_prev[i] + std::max(std::min(dtau * dt, dtau_max * dt), -(dtau_max * dt));
	}

	// save for next iteration and return
	tau_d_prev = tau_d_sat;
	return tau_d_sat;
}

Eigen::Isometry3d
CartesianPDNullspaceController::filter_desired_pose(const Eigen::Isometry3d& T_ref)
{
	std::lock_guard lock(mtx_T_ref);
;
	auto [pos_ref, ori_ref] = std::tuple{ Eigen::Vector3d(T_ref.translation()), Eigen::Quaterniond(T_ref.rotation()) };
	auto [pos_d, ori_d] = std::tuple{ Eigen::Vector3d(T_d.translation()), Eigen::Quaterniond(T_d.rotation()) };

	pos_d = slew_rate * pos_ref + (1.0 - slew_rate) * pos_d;
	ori_d = ori_d.slerp(slew_rate, ori_ref);

	return Eigen::Translation3d(pos_d) * Eigen::Isometry3d(ori_d);
}

Eigen::Vector6d
CartesianPDNullspaceController::get_pose_error(const Eigen::Isometry3d& T_e, const Eigen::Isometry3d& T_d)
{
	const auto [pos_e, pos_d] = std::tuple{ T_e.translation(), T_d.translation() };
	const auto [R_e, R_d] = std::tuple{ T_e.rotation(), T_d.rotation() };
	// const auto [quat_e, quat_d] = std::tuple{ Eigen::Quaterniond(R_e), Eigen::Quaterniond(R_d) };

	// position error
	const auto pos_de = pos_d - pos_e;

	// orinetation error

	// from (11), (27) in "The Role of Euler Parameters in Robot Control"
	// R_12 = R_01' * R_02 = R_10 * R_02
	// R_12 = (eta_21, eps_1_21) = (eta_21, eps_2_21)

	const auto R_ed = R_e.transpose() * R_d;
	const auto eps_e_de = Eigen::Quaterniond(R_ed).vec();
	const auto eps_de = R_e * eps_e_de; // to base frame

	// error vector
	Eigen::Vector6d x_de;
	x_de << pos_de, eps_de;

	return x_de;
}

void
CartesianPDNullspaceController::callback_command(const geometry_msgs::PoseStampedConstPtr& msg)
{
	// read new transformation
	std::lock_guard lock(mtx_T_ref);
	const auto pose = msg.get()->pose;
	T_ref = Eigen::make_tf(pose);

	// ensure proper orientation
	static auto ori_ref_prev = Eigen::Quaterniond(T_ref.rotation());
	auto ori_ref = Eigen::Quaterniond(T_ref.rotation());
	if (ori_ref_prev.coeffs().dot(ori_ref.coeffs()) < 0.0)
	{
		ori_ref.coeffs() = -ori_ref.coeffs();
		T_ref.linear() = ori_ref.toRotationMatrix(); // set rotation (linear = rotation)
		// T_ref = Eigen::Translation3d(T_ref.translation()) * Eigen::Isometry3d(ori_ref);
	}
	ori_ref_prev = ori_ref;
}

} // namespace franka_controllers

