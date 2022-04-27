#include <franka_controllers/cartesian_admittance_controller.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros_utils/eigen.h>

// export controller
PLUGINLIB_EXPORT_CLASS(franka_controllers::CartesianAdmittanceController, controller_interface::ControllerBase)

namespace franka_controllers
{

bool
CartesianAdmittanceController::init(hardware_interface::RobotHW *hw, ros::NodeHandle& nh)
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
		&CartesianAdmittanceController::callback_command, // function pointer
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
CartesianAdmittanceController::starting(const ros::Time& time)
{
	// set equilibrium point to current state
	T_d = T_ref = get_robot_state().T_e;

	// set desired nullspace to initial robot state
	qN_d = get_robot_state().q;
}

std::tuple<Eigen::Vector6d, Eigen::Vector6d, Eigen::Vector6d>
CartesianAdmittanceController::get_desired(const Eigen::Isometry3d& T_ref)
{
	std::lock_guard lock(mtx_T_ref);

	// current desired pose (member variable)
	// static Eigen::Isometry3d T_d = Eigen::Isometry3d::Identity();

	ROS_WARN_STREAM_ONCE("The controller currently ignores desired twist and acceleration in CartesianAdmittanceController::get_desired().");

	// apply slew rate on reference pose
	auto [p_ref, ori_ref] = std::tuple{ Eigen::Vector3d(T_ref.translation()), Eigen::Quaterniond(T_ref.rotation()) };
	auto [p_d, ori_d] = std::tuple{ Eigen::Vector3d(T_d.translation()), Eigen::Quaterniond(T_d.rotation()) };

	p_d = slew_rate * p_ref + (1.0 - slew_rate) * p_d;
	ori_d = ori_d.slerp(slew_rate, ori_ref);
	T_d = Eigen::Translation3d(p_d) * Eigen::Isometry3d(ori_d);

	// desired pose [p_d, eps_d]
	Eigen::Vector6d x_d;
	x_d << p_d, ori_d.vec();

	// desired twist [dp_d, w_d]  (NOT IMPLEMENTED)
	Eigen::Vector6d dx_d = Eigen::Vector6d::Zero();

	// desired acceleration [ddp_d, dw_d] (NOT IMPLEMENTED)
	Eigen::Vector6d ddx_d = Eigen::Vector6d::Zero();

	// return desired {pose, twist, acceleration}
	return { x_d, dx_d, ddx_d };
}

std::tuple<Eigen::Vector6d, Eigen::Vector6d, Eigen::Vector6d>
CartesianAdmittanceController::spatial_impedance(const Eigen::Vector6d& x_d, const Eigen::Vector6d& dx_d, const Eigen::Vector6d& ddx_d, const Eigen::Vector6d& h_e, double dt)

	// gains
	// [Kp, Ko, Dp, Do, Mp, Mo] defined in cartesian_admittance_controller.h

	// integration using Euler (https://en.wikipedia.org/wiki/Numerical_methods_for_ordinary_differential_equations#Euler_method)
	// y(t + dt) = y(t) + dt * dy(t)
{
	// ----------------------------------------------------------------------------------------------------------

	// end-effector wrench
	auto [f_e, mu_e] = std::tuple{ h_e.head(3), h_e.tail(3) };

	// ----------------------------------------------------------------------------------------------------------

	// translation
	// Mp * ddp_cd + Dp * dp_cd + Kp * p_cd = f_e --> ddp_cd = inv(Mp) * (f_e - Dp * dp_cd - Kp * p_cd)

	static Eigen::Vector3d ddp_cd, dp_cd, p_cd;
	{
		ddp_cd = Mp.inverse() * (f_e - Dp * dp_cd - Kp * p_cd);
		dp_cd = dp_cd + dt * ddp_cd;
		p_cd = p_cd + dt * dp_cd;
	}

	// ----------------------------------------------------------------------------------------------------------

	// orientation
	// Mo * dw_d_cd + Do * w_d_cd + Ko_ * eps_d_cd = mu_d

	// some lambdas
	auto Ad = [](auto& T) { return (Eigen::Matrix6d() << T.rotation(), Eigen::Matrix3d::Zero(), Eigen::skew(T.translation()) * T.rotation(), T.rotation()).finished(); };
	auto E = [](auto& eta, auto& eps) { return eta * Eigen::Matrix3d::Identity() - Eigen::skew(eps); };
	auto exp = [](auto& r){ return Eigen::Quaterniond(Eigen::AngleAxisd(cos(r.norm()), r/r.norm() * sin(r.norm()))); };

	static double eta_cd;
	static Eigen::Vector3d dw_d_cd, w_d_cd, eps_d_cd, mu_d_e;
	static Eigen::Quaterniond quat_d_cd;
	{
		// wrench transform: mu_d_e = [S(t_d0) * R_d0 R_d0] * h_e;
		mu_d_e = Ad(T_d).bottomRows(3) * h_e;

		// integration
		const auto Ko_ = 2 * E(eta_cd, eps_d_cd).transpose() * Ko;
		dw_d_cd = Mo.inverse() * (mu_e - Do * w_d_cd - Ko_ * eps_d_cd);
		w_d_cd = w_d_cd + dt * dw_d_cd;

		// quaternion integration
		quat_d_cd = exp(dt/2 * w_d_cd) * quat_d_cd;
		eps_d_cd = quat_d_cd.vec();
		eta_cd = Eigen::AngleAxisd(quat_d_cd).angle();
	}

	// ----------------------------------------------------------------------------------------------------------

	// compliant frame

	const auto& R_d = T_d.rotation();
	const auto& [p_d, eps_d] = std::tuple{ x_d.head(3), x_d.tail(3) };
	const auto& [dp_d, w_d] = std::tuple{ dx_d.head(3), dx_d.tail(3) };
	const auto& [ddp_d, dw_d] = std::tuple{ ddx_d.head(3), ddx_d.tail(3) };

	// position: p_cd - p_c - p_d --> p_c = p_cd + p_d
	auto p_c = p_cd + p_d;
	auto dp_c = dp_cd + dp_d;
	auto ddp_c = ddp_cd - ddp_d;

	// orientation:
	// auto quat_c = quat_d * quat_d_cd;
	auto eps_c = R_d * eps_d_cd;
	auto w_c = R_d * w_d_cd;
	auto dw_c = R_d * dw_d_cd;

	// compliant vectors
	static Eigen::Vector6d x_c, dx_c, ddx_c;
	x_c << p_c, eps_c;
	dx_c << dp_c, w_c;
	ddx_c << ddp_c, dw_c;

	return { x_c, dx_c, ddx_c };
}

void
CartesianAdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
	// elapsed time
	static ros::Duration elapsed_time{0.};
	const auto dt = period.toSec();
	elapsed_time += period;

	// desired pose, twist and accelereation with applied slew rate filter
	auto [x_d, dx_d, ddx_d] = get_desired(T_ref);

	// read robot state and dynamics
	auto [T_e, q, dq, h_e] = get_robot_state();
	auto [J, dJ, M, C, g] = get_robot_dynamics(q, dq, dt);

	// spatial impedance (compliant frame), x ∈ [6x1]
	auto [x_c, dx_c, ddx_c] = spatial_impedance(x_d, dx_d, ddx_d, h_e, dt);

	// position and orientation control, a ∈ [6x1]
	// auto a = pos_ori_control(x_c, dx_c, ddx_c);

	// desired torque (inverse dynamics)
	// auto tau_d = M(q) * J(q).inverse() * (a - dJ(q, dq) * dq) + C * dq + g;
	// auto tau_m = M * J.inverse() * (a - dJ(q, dq) * dq) + C * dq + g;
	// auto tau_d = tau_m + J.transpose() * h_e;

	// saturate rate-of-effort (rotatum)
	// if (dtau_max > 0)
		// tau_d = saturate_rotatum(tau_d, period.toSec());

	// set desired command on joint handles
	// for (size_t i = 0; i < num_joints; ++i)
		// joint_handles[i].setCommand(tau_d[i]);

	// return;

	// OLD

	// compute errors (in base frame)
	auto dx_e = J * dq;
	const auto x_de = get_pose_error(T_e, T_d); // [p_de, eps_de]
	const auto dx_de = Eigen::Vector6d::Zero() - dx_e; // no desired velocity

	// // cartesian torque
	const auto tau_task = J.transpose() * (kp * x_de + kd * dx_de);

	/// // const auto M_xe = (J * M.inverse() * J.transpose()).inverse();
	// // Eigen::Vector7d u = J.transpose() * M_xe * (kp * x_de + kd * dx_de) + g;

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

	// // apply slew rate (filter) on reference pose
	// T_d = filter_desired_pose(T_ref);
}

CartesianAdmittanceController::RobotState
CartesianAdmittanceController::get_robot_state()
{
	const auto robot_state = state_handle->getRobotState();
	return
	{
		Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())),
		Eigen::Vector7d(robot_state.q.data()),
		Eigen::Vector7d(robot_state.dq.data()),
		Eigen::Vector6d(robot_state.O_F_ext_hat_K.data()) // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame
	};
}


CartesianAdmittanceController::RobotDynamics
CartesianAdmittanceController::get_robot_dynamics(const Eigen::Vector7d& q, const Eigen::Vector7d& dq, double dt)
{
	RobotDynamics dyn =
	{
		.J = Eigen::Matrix6x7d::Map(model_handle->getZeroJacobian(franka::Frame::kEndEffector).data()),
		.dJ = Eigen::Matrix6x7d(),
		.M = Eigen::Matrix7d::Map(model_handle->getMass().data()),
		.C = Eigen::Vector7d(model_handle->getCoriolis().data()),
		.g = Eigen::Vector7d(model_handle->getGravity().data())
	};

	// jacobian derivative
	static auto J_prev = dyn.J;
	dyn.dJ = (dyn.J - J_prev) / dt;
	J_prev = dyn.J;

	return dyn;
}

Eigen::Vector7d
CartesianAdmittanceController::saturate_rotatum(const Eigen::Vector7d& tau_d, const double dt)
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
CartesianAdmittanceController::filter_desired_pose(const Eigen::Isometry3d& T_ref)
{
	std::lock_guard lock(mtx_T_ref);

	auto [pos_ref, ori_ref] = std::tuple{ Eigen::Vector3d(T_ref.translation()), Eigen::Quaterniond(T_ref.rotation()) };
	auto [pos_d, ori_d] = std::tuple{ Eigen::Vector3d(T_d.translation()), Eigen::Quaterniond(T_d.rotation()) };

	pos_d = slew_rate * pos_ref + (1.0 - slew_rate) * pos_d;
	ori_d = ori_d.slerp(slew_rate, ori_ref);

	return Eigen::Translation3d(pos_d) * Eigen::Isometry3d(ori_d);
}

Eigen::Vector6d
CartesianAdmittanceController::get_pose_error(const Eigen::Isometry3d& T_e, const Eigen::Isometry3d& T_d)
{
	const auto [pos_e, pos_d] = std::tuple{ T_e.translation(), T_d.translation() };
	const auto [R_e, R_d] = std::tuple{ T_e.rotation(), T_d.rotation() };
	// const auto [quat_e, ori_d] = std::tuple{ Eigen::Quaterniond(R_e), Eigen::Quaterniond(R_d) };

	// position error
	const auto pos_de = pos_d - pos_e;

	// orinetation error

	// from (11), (27) in "The Role of Euler Parameters in Robot Control"
	// R_12 = R_01' * R_02 = R_10 * R_02
	// R_12 = (eta_21, eps_1_21) = (eta_21, eps_2_21)

	const auto R_ed = R_e.transpose() * R_d;
	const auto eps_e_de = Eigen::Quaterniond(R_ed).vec();
	const auto eps_de = R_e * eps_e_de; // to base frame

	// error vector [6 x 1]
	Eigen::Vector6d x_de;
	x_de << pos_de, eps_de;

	return x_de;
}

void
CartesianAdmittanceController::callback_command(const geometry_msgs::PoseStampedConstPtr& msg)
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

