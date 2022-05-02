#include <franka_controllers/cartesian_admittance_controller.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.hpp>
#include <eigen_conversions/eigen_msg.h>
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

	// read gains from config
	std::vector<double> vec_Kp, vec_Ko, vec_Dp, vec_Do, vec_Mp, vec_Mo, vec_kp, vec_kd;
	if (
		not nh.getParam("Kp", vec_Kp) or
		not nh.getParam("Ko", vec_Ko) or
		not nh.getParam("Dp", vec_Dp) or
		not nh.getParam("Do", vec_Do) or
		not nh.getParam("Mp", vec_Mp) or
		not nh.getParam("Mo", vec_Mo) or
		not nh.getParam("kpp", kpp) or
		not nh.getParam("kpo", kpo) or
		not nh.getParam("kvp", kvp) or
		not nh.getParam("kvo", kvo) or
		not nh.getParam("kn", kn) or
		// not nh.getParam("kc", kc) or
		not nh.getParam("kp", vec_kp) or
		not nh.getParam("kd", vec_kd) or
		not nh.getParam("dtau_max", dtau_max) or
		not nh.getParam("slew_rate", slew_rate)
	)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Error reading config parameters.");
		return false;
	}
	else // convert vectors to Eigen matrices
	{
		Kp = Eigen::Vector3d(vec_Kp.data()).asDiagonal();
		Ko = Eigen::Vector3d(vec_Ko.data()).asDiagonal();
		Dp = Eigen::Vector3d(vec_Dp.data()).asDiagonal();
		Do = Eigen::Vector3d(vec_Do.data()).asDiagonal();
		Mp = Eigen::Vector3d(vec_Mp.data()).asDiagonal();
		Mo = Eigen::Vector3d(vec_Mo.data()).asDiagonal();
		kp = Eigen::Vector6d(vec_kp.data()).asDiagonal();
		kd = Eigen::Vector6d(vec_kd.data()).asDiagonal();
	}

	// subscribe to command topic for desired pose
	sub_command = nh.subscribe<geometry_msgs::PoseStamped>(
		"command", // topic name
		20, // queue size
		&CartesianAdmittanceController::callback_command, // function pointer
		this,
		ros::TransportHints().reliable().tcpNoDelay() // transport hints
	);

	// debug publisher + worker
	pub_msg_debug = nh.advertise<CartesianAdmittanceControllerDebug>("debug", 1);
	thread_msg_debug = std::thread([&]()
	{
		ros::Rate r(100); // hz
		while (ros::ok())
		{
			mtx_msg_debug.lock();
			pub_msg_debug.publish(msg_debug);
			mtx_msg_debug.unlock();
			r.sleep();
		}
	});

	// init complete
	ROS_WARN_STREAM("Initialized " << CONTROLLER_NAME << " with:\n\n"
		<< "Kp = "        << Kp.diagonal().transpose() << "\n"
		<< "Ko = "        << Ko.diagonal().transpose() << "\n"
		<< "Dp = "        << Dp.diagonal().transpose() << "\n"
		<< "Do = "        << Do.diagonal().transpose() << "\n"
		<< "Mp = "        << Mp.diagonal().transpose() << "\n"
		<< "Mo = "        << Mo.diagonal().transpose() << "\n"
		<< "kp = "        << kp.diagonal().transpose() << "\n"
		<< "kd = "        << kd.diagonal().transpose() << "\n"
		<< "kpp = "       << kpp << "\n"
		<< "kpo = "       << kpo << "\n"
		<< "kvp = "       << kvp << "\n"
		<< "kvo = "       << kvo << "\n"
		<< "kn = "        << kn << "\n"
		// << "kc = "        << kc << "\n"
		<< "kc = "        << "2 * sqrt(kn)" << "\n"
		<< "dtau_max = "  << dtau_max << "\n"
		<< "slew_rate = " << slew_rate  << "\n"
		<< "tau_ext_lowpass_filter = " << ros::param::param<double>("/tau_ext_lowpass_filter", INFINITY) << "\n"
	);

	return true;
}

void
CartesianAdmittanceController::starting(const ros::Time& time)
{
	// set equilibrium point to current state
	pose_d = { get_robot_state().p_e, get_robot_state().R_e };
	buffer_pose_ref.writeFromNonRT(pose_d);

	// set desired nullspace to initial robot state
	qN_d = get_robot_state().q;

	auto T_d = Eigen::Translation3d(std::get<0>(pose_d)) * Eigen::Isometry3d(std::get<1>(pose_d));
	ROS_WARN_STREAM("setting desired pose to:"
		<< "\n\n" << T_d.matrix()
		<< "\n\nRPY: " << T_d.rotation().eulerAngles(0, 1, 2).transpose()
		<< "\n\n[eta, eps]: [" << Eigen::Quaterniond(T_d.rotation()).w() << ", " << Eigen::Quaterniond(T_d.rotation()).vec().transpose() << "]"
		<< "\n\n[x, y, z, w]: " << Eigen::Quaterniond(T_d.rotation()).coeffs().transpose()
	);

	// give some time to read, lol
	sleep(2);
}

void
CartesianAdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
	// controller loop
	// pose, twist etc. are given by p ∈ [3x1], R ∈ [3x3], w ∈ [3x1]

	// elapsed time
	static ros::Duration elapsed_time{0.};
	const auto dt = period.toSec();
	elapsed_time += period;

	// update dynamics parameters
	// this->dynamic_reconfigure_parameters();

	// read reference pose [p, R]
	auto [p_ref, R_ref] = *buffer_pose_ref.readFromRT();

	// desired pose, twist and accelereation with applied slew rate filter
	auto [p_d, R_d, dp_d, w_d, ddp_d, dw_d] = get_desired(p_ref, R_ref);

	// read robot state and dynamics
	auto [p_e, R_e, q, dq, h_e] = get_robot_state();
	auto [J, dJ, M, c, g] = get_robot_dynamics(q, dq, dt);
	auto [dp_e, w_e] = std::tuple{ (J * dq).head(3), (J * dq).tail(3) };

	// spatial impedance (compliant frame)
	auto [p_c, R_c, dp_c, w_c, ddp_c, dw_c] = spatial_impedance(p_d, R_d, dp_d, w_d, ddp_d, dw_d, h_e, dt);

	// position and orientation control, a ∈ [6x1]
	auto a = pos_ori_control(p_c, p_e, R_c, R_e, dp_c, dp_e, w_c, w_e, ddp_c, dw_c);

	// desired torque (inverse dynamics)
	// Eigen::Matrix7x6d J_pinv = (M.inverse() * J.transpose()) * (J * M.inverse() * J.transpose()).inverse(); // shiet
	// Eigen::Matrix7x6d J_pinv = Eigen::pseudo_inverse(J, 0.2);
	// Eigen::Vector7d tau_m = M * J_pinv * (a - dJ * dq) + c + g;

	// spatial impedance for redundant manipulators (13), (9)
	// Eigen::Vector7d phi_n = tau_null;
	// Eigen::Vector7d tau_d = M * (J_pinv * (a - dJ * dq) + phi_n) + c + g + J.transpose() * h_e;

	// compute errors (in base frame)
	// x = [p, eps], dx = [dp, w]

	auto p_ce = p_c - p_e;
	auto R_ec = R_e.transpose() * R_c;
	auto eps_e_ce = Eigen::Vector3d(Eigen::Quaterniond(R_ec).vec()); // must explitcly state Vector3d, otherwise blyat
	auto eps_ce = R_e * eps_e_ce; // to base frame
	auto x_ce = (Eigen::Vector6d() << p_ce, eps_ce).finished();

	auto dx_e = J * dq;
	auto dx_c = (Eigen::Vector6d() << dp_c, w_c).finished();
	auto dx_ce = dx_c - dx_e;

	// auto [T_e, T_d, T_c] = std::tuple{ Eigen::Translation3d(p_e) * Eigen::Isometry3d(R_e), Eigen::Translation3d(p_d) * Eigen::Isometry3d(R_d), Eigen::Translation3d(p_c) * Eigen::Isometry3d(R_c) };
	// auto x_err = get_pose_error(T_e, T_c); // [p_err, eps_err]
	// auto dx_err = Eigen::Vector6d::Zero() - dx_e; // no desired velocity

	// cartesian torque
	auto tau_task = J.transpose() * (kp * x_ce + kd * dx_ce);
	// auto tau_task = J.transpose() * (kp * x_err + kd * dx_err);

	// nullspace torque (https://studywolf.wordpress.com/2013/09/17/robot-control-5-controlling-in-the-null-space/)
	auto J_T_pinv = Eigen::pseudo_inverse(J.transpose(), 0.2); // damped pseudo inverse
	auto I7x7d = Eigen::Matrix7d::Identity();
	auto kc = 2.0 * sqrt(kn); // damping ratio of 1.0
	auto tau_null = (I7x7d - J.transpose() * J_T_pinv) * (kn * (qN_d - q) - kc * dq);

	// desired joint torque
	Eigen::Vector7d tau_d = tau_task + tau_null + c;
	// Eigen::Vector7d tau_d = tau_m + tau_null + J.transpose() * h_e;

	// saturate rate-of-effort (rotatum)
	if (dtau_max > 0)
		tau_d = saturate_rotatum(tau_d, period.toSec());

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		joint_handles[i].setCommand(tau_d[i]);

	// ROS_INFO_STREAM("\n");
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
CartesianAdmittanceController::get_desired(const Eigen::Vector3d& p_ref, const Eigen::Matrix3d& R_ref)
{
	// apply slew rate on reference pose
	ROS_WARN_STREAM_ONCE("The controller currently ignores desired twist and acceleration in CartesianAdmittanceController::get_desired().");

	// current desired pose (member variable)
	auto& [p_d, R_d] = pose_d;
	auto [quat_d, quat_ref] = std::tuple{ Eigen::Quaterniond(R_d), Eigen::Quaterniond(R_ref) };

	p_d = slew_rate * p_ref + (1.0 - slew_rate) * p_d;
	quat_d = quat_d.slerp(slew_rate, quat_ref);
	R_d = quat_d.toRotationMatrix();

	// desired twist [dp_d, w_d]
	// NOT IMPLEMENTED
	Eigen::Vector6d dx_d = Eigen::Vector6d::Zero();

	// desired acceleration [ddp_d, dw_d]
	// NOT IMPLEMENTED
	Eigen::Vector6d ddx_d = Eigen::Vector6d::Zero();

	// return desired {pose, twist, acceleration}
	return { p_d, R_d, dx_d.head(3), dx_d.tail(3), ddx_d.head(3), ddx_d.tail(3) };
}

CartesianAdmittanceController::RobotState
CartesianAdmittanceController::get_robot_state()
{
	const auto robot_state = state_handle->getRobotState();
	const auto T_e = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

	// auto J = Eigen::Matrix6x7d::Map(model_handle->getZeroJacobian(franka::Frame::kEndEffector).data());
	// Eigen::Vector7d tau_ext = Eigen::Vector7d(robot_state.tau_ext_hat_filtered.data());
	// Eigen::Vector6d h_e = Eigen::pseudo_inverse(J.transpose(), 0.2) * tau_ext;

	return
	{
		T_e.translation(),
		T_e.rotation(),
		Eigen::Vector7d(robot_state.q.data()),
		Eigen::Vector7d(robot_state.dq.data()),
		Eigen::Vector6d(robot_state.O_F_ext_hat_K.data()) // estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame
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
		.c = Eigen::Vector7d(model_handle->getCoriolis().data()), // c = C*dq
		.g = Eigen::Vector7d(model_handle->getGravity().data())
	};

	// jacobian derivative
	static auto J_prev = dyn.J;
	dyn.dJ = (dyn.J - J_prev) / dt;
	J_prev = dyn.J;

	return dyn;
}

CartesianAdmittanceController::Frame
CartesianAdmittanceController::spatial_impedance(const Eigen::Vector3d& p_d, const Eigen::Matrix3d& R_d, const Eigen::Vector3d& dp_d, const Eigen::Vector3d& w_d, const Eigen::Vector3d& ddp_d, const Eigen::Vector3d& dw_d, const Eigen::Vector6d& h_e, double dt)
{

	// compute compliant frame based on desired frame, end-effector wrench and gains [Kp, Ko, Dp, Do, Mp, Mo]
	// using Euler integration: "y(t + dt) = y(t) + dt * dy(t)" and quaternion integration: ""
	// (https://en.wikipedia.org/wiki/Numerical_methods_for_ordinary_differential_equations#Euler_method)

	// end-effector wrench
	auto [f_e, mu_e] = std::tuple{ h_e.head(3), h_e.tail(3) };

	// translation
	// Mp * ddp_cd + Dp * dp_cd + Kp * p_cd = f_e

	static Eigen::Vector3d ddp_cd, dp_cd, p_cd;
	{
		ddp_cd = Mp.inverse() * (f_e - Dp * dp_cd - Kp * p_cd);
		dp_cd = dp_cd + dt * ddp_cd;
		p_cd = p_cd + dt * dp_cd;
	}

	// orientation
	// Mo * dw_d_cd + Do * w_d_cd + Ko_ * eps_d_cd = mu_d

	// some lambdas
	auto E = [](const auto& eta, const auto& eps) { return eta * Eigen::Matrix3d::Identity() - Eigen::skew(eps); };
	auto Ad = [](const auto& T) { return (Eigen::Matrix6d() << T.rotation(), Eigen::Matrix3d::Zero(), Eigen::skew(T.translation()) * T.rotation(), T.rotation()).finished(); };
	auto exp = [](const auto& r) { auto [eta, eps] = std::tuple{ std::cos(r.norm()), (r / r.norm()) * std::sin(r.norm()) }; return Eigen::Quaterniond(eta, eps[0], eps[1], eps[2]); };

	static Eigen::Vector3d w_d_cd{}, dw_d_cd{};
	static Eigen::Quaterniond quat_d_cd = Eigen::Quaterniond::Identity(); // q = {eta, eps[]}
	const auto& [eta_cd, eps_d_cd] = std::tuple{ quat_d_cd.coeffs().w(), quat_d_cd.vec() };
	{
		// wrench transform: mu_d_e = [S(t_d0) * R_d0 R_d0] * h_e;
		auto T_d = Eigen::Translation3d(p_d) * Eigen::Isometry3d(R_d);
		auto mu_d_e = Ad(T_d).bottomRows(3) * h_e;

		// integration
		const auto Ko_ = 2 * E(eta_cd, eps_d_cd).transpose() * Ko;
		dw_d_cd = Mo.inverse() * (mu_d_e - Do * w_d_cd - Ko_ * eps_d_cd);
		w_d_cd = w_d_cd + dt * dw_d_cd;

		// quaternion integration
		quat_d_cd = exp(dt/2 * w_d_cd) * quat_d_cd;
	}

	// compliant frame

	// position:
	// p_cd = p_c - p_d --> p_c = p_cd + p_d

	auto p_c = p_cd + p_d;
	auto dp_c = dp_cd + dp_d;
	auto ddp_c = ddp_cd - ddp_d;

	// orientation:
	// q_d_cd = ..

	auto quat_d = Eigen::Quaterniond(R_d);
	auto quat_c = quat_d * quat_d_cd;
	auto R_c = quat_c.toRotationMatrix();
	auto w_c = R_d * w_d_cd;
	auto dw_c = R_d * dw_d_cd;

	// update debug info
	if (mtx_msg_debug.try_lock())
	{
		tf::wrenchEigenToMsg(h_e, msg_debug.h_e);
		// tf::vectorEigenToMsg(mu_d_e, msg_debug.mu_d_e);
		tf::vectorEigenToMsg(p_d, msg_debug.p_d);
		tf::vectorEigenToMsg(p_c, msg_debug.p_c);
		tf::vectorEigenToMsg(quat_d.vec(), msg_debug.eps_d);
		tf::vectorEigenToMsg(quat_c.vec(), msg_debug.eps_c);
		mtx_msg_debug.unlock();
	}

	return { p_c, R_c, dp_c, w_c, ddp_c, dw_c };
}

Eigen::Vector6d
CartesianAdmittanceController::pos_ori_control(const Eigen::Vector3d& p_c, const Eigen::Vector3d& p_e, const Eigen::Matrix3d& R_c, const Eigen::Matrix3d& R_e, const Eigen::Vector3d& dp_c, const Eigen::Vector3d& dp_e, const Eigen::Vector3d& w_c, const Eigen::Vector3d& w_e, const Eigen::Vector3d& ddp_c, const Eigen::Vector3d& dw_c)
{
	// compute error between compliant and end-effector frame

	// position error
	auto p_ce = p_c - p_e;
	auto dp_ce = dp_c - dp_e;

	// orientation error
	auto R_ec = R_e.transpose() * R_c;
	auto eps_e_ce = Eigen::Vector3d(Eigen::Quaterniond(R_ec).vec()); // must explitcly state Vector3d, otherwise blyat
	auto eps_ce = R_e * eps_e_ce; // to base frame
	auto w_ce = w_c - w_e;

	// desired acclerations
	Eigen::Vector3d a_p = ddp_c + kvp * dp_c + kpp * p_c;
	Eigen::Vector3d a_o = dw_c + kvo * w_ce + kpo * eps_ce;

	return (Eigen::Vector6d() << a_p, a_o).finished();
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

Eigen::Vector6d
CartesianAdmittanceController::get_pose_error(const Eigen::Isometry3d& T_e, const Eigen::Isometry3d& T_d)
{
	auto [p_e, p_d] = std::tuple{ T_e.translation(), T_d.translation() };
	auto [R_e, R_d] = std::tuple{ T_e.rotation(), T_d.rotation() };

	// position error
	auto p_de = p_d - p_e;

	// orientation error

	// from (11), (27) in "The Role of Euler Parameters in Robot Control"
	// R_12 = R_01' * R_02 = R_10 * R_02
	// R_12 = (eta_21, eps_1_21) = (eta_21, eps_2_21)

	auto R_ed = R_e.transpose() * R_d;
	auto eps_e_de = Eigen::Vector3d(Eigen::Quaterniond(R_ed).vec());
	auto eps_de = R_e * eps_e_de; // to base frame

	// error vector [6 x 1]
	Eigen::Vector6d x_de;
	x_de << p_de, eps_de;

	return x_de;
}

void
CartesianAdmittanceController::callback_command(const geometry_msgs::PoseStampedConstPtr& msg)
{
	// read new transformation
	const auto pose_ref = msg.get()->pose;
	auto T_ref = Eigen::make_tf(pose_ref);

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

	// write pose
	buffer_pose_ref.writeFromNonRT(Pose{ T_ref.translation(), T_ref.rotation() });
}

} // namespace franka_controllers

