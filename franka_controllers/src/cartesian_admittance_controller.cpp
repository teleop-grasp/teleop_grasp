#include <franka_controllers/cartesian_admittance_controller.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros_utils/eigen.h>
#include <ros_utils/std.h>

#ifdef NDEBUG
constexpr auto DEBUG_MODE = false;
#else
constexpr auto DEBUG_MODE = true;
#endif

// export controller
PLUGINLIB_EXPORT_CLASS(franka_controllers::CartesianAdmittanceController, controller_interface::ControllerBase)

namespace franka_controllers
{

bool
CartesianAdmittanceController::init(hardware_interface::RobotHW *hw, ros::NodeHandle& nh)
{

	// check debug/release mode
	if (DEBUG_MODE)
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << " is running in DEBUG mode and may no suffice 1 kHz update loop.");
		sleep(5);
	}

	// determine whether in simulation or on real robot
	std::vector<std::string> vec_nodes;
	in_simulation = ros::master::getNodes(vec_nodes) and not is_in("/franka_control_node", vec_nodes);

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
		// not nh.getParam("kp", vec_kp) or
		// not nh.getParam("kd", vec_kd) or
		not nh.getParam("dtau_max", dtau_max) or
		not nh.getParam("slew_rate", slew_rate) or
		not nh.getParam("state_publish_rate", state_publish_rate)
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
		// kp = Eigen::Vector6d(vec_kp.data()).asDiagonal();
		// kd = Eigen::Vector6d(vec_kd.data()).asDiagonal();
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
	pub_msg_T_c = nh.advertise<geometry_msgs::PoseStamped>("T_c", 1);
	msg_debug.T_c.header.frame_id = "panda_link0";
	thread_msg_debug = std::thread([&]()
	{
		ros::Rate r(state_publish_rate); // hz
		while (ros::ok())
		{
			mtx_msg_debug.lock();
			pub_msg_debug.publish(msg_debug);
			pub_msg_T_c.publish(msg_debug.T_c);
			mtx_msg_debug.unlock();
			r.sleep();
		}
	});

	// init complete
	ROS_WARN_STREAM("Initialized " << CONTROLLER_NAME << " with:\n\n"
		<< "in_simulation = "          << std::boolalpha << in_simulation << "\n"
		<< "Kp = "                     << Kp.diagonal().transpose() << "\n"
		<< "Ko = "                     << Ko.diagonal().transpose() << "\n"
		<< "Dp = "                     << Dp.diagonal().transpose() << "\n"
		<< "Do = "                     << Do.diagonal().transpose() << "\n"
		<< "Mp = "                     << Mp.diagonal().transpose() << "\n"
		<< "Mo = "                     << Mo.diagonal().transpose() << "\n"
		// << "kp = "                     << kp.diagonal().transpose() << "\n"
		// << "kd = "                     << kd.diagonal().transpose() << "\n"
		<< "kpp = "                    << kpp << "\n"
		<< "kpo = "                    << kpo << "\n"
		<< "kvp = "                    << kvp << "\n"
		<< "kvo = "                    << kvo << "\n"
		<< "kn = "                     << kn << "\n"
		// << "kc = "                     << kc << "\n"
		<< "kc = "                     << "2 * sqrt(kn)" << "\n"
		<< "dtau_max = "               << dtau_max << "\n"
		<< "slew_rate = "              << slew_rate  << "\n"
		<< "state_publish_rate = "     << state_publish_rate  << "\n"
		<< "tau_ext_lowpass_filter = " << ros::param::param<double>("/tau_ext_lowpass_filter", INFINITY) << "\n"
		<< "\n"
	);

	return true;
}

void
CartesianAdmittanceController::starting(const ros::Time& /* time */)
{
	const franka::RobotState robot_state = state_handle->getRobotState();

	// set equilibrium point to current state
	pose_d = Pose{ get_robot_state().p_e, get_robot_state().R_e };
	buffer_pose_ref.initRT(pose_d);
	Eigen::Isometry3d T_d = Eigen::Translation3d(std::get<0>(pose_d)) * Eigen::Isometry3d(std::get<1>(pose_d));

	// set desired nullspace to initial robot state
	qN_d = get_robot_state().q;

	// read defined transformations
	// Eigen::Isometry3d O_T_EE = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Isometry3d EE_T_K = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.EE_T_K.data()));
	// Eigen::Isometry3d NE_T_EE = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.NE_T_EE.data()));
	// Eigen::Isometry3d F_T_NE = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.F_T_NE.data()));
	// Eigen::Isometry3d O_T_NE = O_T_EE * NE_T_EE.inverse();

	// print informations
	ROS_INFO_STREAM("Initial configuration:\n"
		<< "qN_d: [" << qN_d.transpose() << "]\n"
		<< "p_d: [ " << T_d.translation().transpose() << "]\n"
		<< "[eta_d, eps_d]: [" << Eigen::Quaterniond(T_d.rotation()).w() << ", " << Eigen::Quaterniond(T_d.rotation()).vec().transpose() << "]\n"
		<< "RPY: [" << T_d.rotation().eulerAngles(0, 1, 2).transpose() << "]\n"
		<< "T_d:\n\n" << T_d.matrix() << "\n"
		<< "T_K:\n\n" << EE_T_K.matrix() << "\n"
		<< "\n"
	);

	// ROS_INFO_STREAM("Currently defined transformations:\n\n");
	// std::cout << "O_T_EE:\n\n" << O_T_EE.matrix() << std::endl;
	// std::cout << "EE_T_K:\n\n" << EE_T_K.matrix() << std::endl;
	// std::cout << "O_T_NE:\n\n" << O_T_NE.matrix() << std::endl;
	// std::cout << "F_T_NE:\n\n" << F_T_NE.matrix() << std::endl;

	sleep(2);
}

void
CartesianAdmittanceController::update(const ros::Time& /* time */, const ros::Duration& period)
{
	// controller loop
	// pose, twist etc. are given by p ∈ [3x1], R ∈ [3x3], w ∈ [3x1]
	// FrankaHWSim will automatically compensate gravity (https://frankaemika.github.io/docs/franka_ros.html?highlight=gravity#jointstateinterface)

	// elapsed time
	static ros::Duration elapsed_time{0.};
	double dt = (period.toSec() < 1e-7) ? 0.001 : period.toSec(); // ensure dt > 0
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
	Eigen::Vector6d dx_e = J * dq;
	// auto [dp_e, w_e] = std::tuple{ dx_e.head(3), dx_e.tail(3) };
	auto [dp_e, w_e] = std::tuple{ Eigen::Vector3d(dx_e.head(3)), Eigen::Vector3d(dx_e.tail(3)) };

	// spatial impedance (compliant frame)
	auto [p_c, R_c, dp_c, w_c, ddp_c, dw_c] = spatial_impedance(p_d, R_d, dp_d, w_d, ddp_d, dw_d, h_e, dt);

	// position and orientation control, a ∈ [6x1]
	// Eigen::Vector6d a = pos_ori_control(p_d, p_e, R_d, R_e, dp_d, dp_e, w_d, w_e, ddp_d, dw_d); // track desired frame
	Eigen::Vector6d a = pos_ori_control(p_c, p_e, R_c, R_e, dp_c, dp_e, w_c, w_e, ddp_c, dw_c);

	// desired task-space torque (inverse dynamics)
	// Eigen::Matrix7x6d J_pinv = Eigen::pseudo_inverse(J, 0.2);
	Eigen::Matrix7x6d J_pinv = M.inverse() * J.transpose() * (J * M.inverse() * J.transpose()).inverse(); // dynamically consistent pseudo-inverse
	Eigen::Vector7d tau_task = M * J_pinv * (a - dJ * dq);
	// Eigen::Vector7d tau_task = M * J_pinv * (a - dJ * dq) + J.transpose() * h_e;

	// nullspace torque
	// (https://studywolf.wordpress.com/2013/09/17/robot-control-5-controlling-in-the-null-space/)
	Eigen::Matrix6x7d J_T_pinv = Eigen::pseudo_inverse(J.transpose(), 0.2); // damped pseudo inverse
	Eigen::Matrix7d I7x7d = Eigen::Matrix7d::Identity();
	double kc = 2.0 * sqrt(kn); // damping ratio of 1.0
	Eigen::Vector7d tau_null = (I7x7d - J.transpose() * J_T_pinv) * (kn * (qN_d - q) - kc * dq);

	// desired joint torque
	// Eigen::Vector7d tau_d = tau_task + c;
	Eigen::Vector7d tau_d = tau_task + tau_null + c;

	// saturate rate-of-effort (rotatum)
	if (dtau_max > 0)
		tau_d = saturate_rotatum(tau_d, dt);

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		joint_handles[i].setCommand(tau_d[i]);

	// update debug info
	if (mtx_msg_debug.try_lock())
	{
		msg_debug.tau_d.data = std::vector<double>(tau_d.data(), tau_d.data() + tau_d.size());
		mtx_msg_debug.unlock();
	}

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
	const franka::RobotState robot_state = state_handle->getRobotState();
	Eigen::Isometry3d T_e = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

	// Eigen::Matrix6x7d J = Eigen::Matrix6x7d::Map(model_handle->getZeroJacobian(franka::Frame::kEndEffector).data());
	// Eigen::Vector7d tau_ext = Eigen::Vector7d(robot_state.tau_ext_hat_filtered.data());
	// Eigen::Vector6d h_e = Eigen::pseudo_inverse(J.transpose(), 0.2) * tau_ext;
	Eigen::Vector6d h_e = Eigen::Vector6d(robot_state.O_F_ext_hat_K.data());
	h_e *= (in_simulation) ? 1 : -1; // invert if running on real robot

	return
	{
		T_e.translation(),
		T_e.rotation(),
		Eigen::Vector7d(robot_state.q.data()),
		Eigen::Vector7d(robot_state.dq.data()),
		h_e // estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame
	};
}

CartesianAdmittanceController::RobotDynamics
CartesianAdmittanceController::get_robot_dynamics(const Eigen::Vector7d& /* q */, const Eigen::Vector7d&  /* dq */, double dt)
{
	RobotDynamics dyn =
	{
		Eigen::Matrix6x7d::Map(model_handle->getZeroJacobian(franka::Frame::kEndEffector).data()), // J
		Eigen::Matrix6x7d::Zero(), // dJ
		Eigen::Matrix7d::Map(model_handle->getMass().data()), // M
		Eigen::Vector7d(model_handle->getCoriolis().data()), // c = C*dq
		Eigen::Vector7d(model_handle->getGravity().data()) // g
	};

	// jacobian derivative
	static Eigen::Matrix6x7d J_prev = dyn.J;
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
	// auto [f_e, mu_e] = std::tuple{ h_e.head(3), h_e.tail(3) };
	auto [f_e, mu_e] = std::tuple{ Eigen::Vector3d(h_e.head(3)), Eigen::Vector3d(h_e.tail(3)) };

	// translation
	// Mp * ddp_cd + Dp * dp_cd + Kp * p_cd = f_e

	// static Eigen::Vector3d p_cd{}, dp_cd{}, ddp_cd{};
	static Eigen::Vector3d p_cd = Eigen::Vector3d::Zero();
	static Eigen::Vector3d dp_cd = Eigen::Vector3d::Zero();
	static Eigen::Vector3d ddp_cd = Eigen::Vector3d::Zero();
	{
		ddp_cd = Mp.inverse() * (f_e - Dp * dp_cd - Kp * p_cd);
		dp_cd = dp_cd + dt * ddp_cd;
		p_cd = p_cd + dt * dp_cd;
	}

	// orientation
	// Mo * dw_d_cd + Do * w_d_cd + Ko_ * eps_d_cd = mu_d

	// some lambdas
	auto E = [](double eta, const Eigen::Vector3d& eps) { return Eigen::Matrix3d(eta * Eigen::Matrix3d::Identity() - Eigen::skew(eps)); };
	auto Ad = [](const Eigen::Isometry3d& T) { return (Eigen::Matrix6d() << T.rotation(), Eigen::Matrix3d::Zero(), Eigen::skew(T.translation()) * T.rotation(), T.rotation()).finished(); };
	auto exp = [](const Eigen::Vector3d& r)
	{
		auto [eta, eps] = std::tuple{ std::cos(r.norm()), Eigen::Vector3d( (r / (r.norm() + 1e-7) * std::sin(r.norm()) )) };
		return Eigen::Quaterniond(eta, eps.x(), eps.y(), eps.z()); // [w, x, y, z]
	};

	// static Eigen::Vector3d w_d_cd{}, dw_d_cd{};
	static Eigen::Vector3d w_d_cd = Eigen::Vector3d::Zero();
	static Eigen::Vector3d dw_d_cd = Eigen::Vector3d::Zero();
	static Eigen::Quaterniond quat_d_cd = Eigen::Quaterniond::Identity(); // q = {eta, eps[]}
	// const auto& [eta_cd, eps_d_cd] = std::tuple{ quat_d_cd.coeffs().w(), quat_d_cd.vec() };
	double eta_cd = quat_d_cd.coeffs().w();
	Eigen::Vector3d eps_d_cd = quat_d_cd.vec();
	{
		// wrench transform: mu_d_e = [S(t_d0) * R_d0 R_d0] * h_e;
		// Eigen::Isometry3d T_d = Eigen::Translation3d(p_d) * Eigen::Isometry3d(R_d);
		Eigen::Isometry3d T_d0 = (Eigen::Translation3d(p_d) * Eigen::Isometry3d(R_d)).inverse();
		Eigen::Vector3d mu_d_e = Ad(T_d0).bottomRows(3) * h_e;

		// integration
		Eigen::Matrix3d Ko_ = 2 * E(eta_cd, eps_d_cd).transpose() * Ko;
		dw_d_cd = Mo.inverse() * (mu_d_e - Do * w_d_cd - Ko_ * eps_d_cd);
		w_d_cd = w_d_cd + dt * dw_d_cd;

		// quaternion integration
		quat_d_cd = exp(dt/2. * w_d_cd) * quat_d_cd;
	}

	// compliant frame

	// position:
	// p_cd = p_c - p_d --> p_c = p_cd + p_d

	Eigen::Vector3d p_c = p_cd + p_d;
	Eigen::Vector3d dp_c = dp_cd + dp_d;
	Eigen::Vector3d ddp_c = ddp_cd + ddp_d;

	// orientation:
	// q_d_cd = ...

	Eigen::Quaterniond quat_d = Eigen::Quaterniond(R_d);
	Eigen::Quaterniond quat_c = quat_d * quat_d_cd;
	Eigen::Matrix3d R_c = quat_c.toRotationMatrix();

	Eigen::Vector3d w_c = (R_d * w_d_cd) + w_d;
	Eigen::Vector3d dw_c = (R_d * dw_d_cd) + dw_d;

	// update debug info
	if (mtx_msg_debug.try_lock())
	{
		tf::wrenchEigenToMsg(h_e, msg_debug.h_e);
		// tf::vectorEigenToMsg(mu_d_e, msg_debug.mu_d_e);
		tf::vectorEigenToMsg(p_d, msg_debug.p_d);
		tf::poseEigenToMsg(Eigen::Isometry3d(Eigen::Translation3d(p_c) * Eigen::Isometry3d(R_c)), msg_debug.T_c.pose);
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
	Eigen::Vector3d p_ce = p_c - p_e;
	Eigen::Vector3d dp_ce = dp_c - dp_e;

	// orientation error
	Eigen::Matrix3d R_ec = R_e.transpose() * R_c;
	Eigen::Vector3d eps_e_ce = Eigen::Quaterniond(R_ec).vec();
	Eigen::Vector3d eps_ce = R_e * eps_e_ce; // to base frame
	Eigen::Vector3d w_ce = w_c - w_e;

	// desired acclerations
	Eigen::Vector3d a_p = ddp_c + kvp * dp_ce + kpp * p_ce;
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
	for (auto i = 0; i < tau_d_sat.size(); ++i)
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
	// const auto& [p_e, p_d] = std::tuple{ T_e.translation(), T_d.translation() };
	// const auto& [R_e, R_d] = std::tuple{ T_e.rotation(), T_d.rotation() };
	Eigen::Vector3d p_e = T_e.translation();
	Eigen::Vector3d p_d = T_d.translation();
	Eigen::Matrix3d R_e = T_e.rotation();
	Eigen::Matrix3d R_d = T_d.rotation();

	// position error
	Eigen::Vector3d p_de = p_d - p_e;

	// orientation error

	// from (11), (27) in "The Role of Euler Parameters in Robot Control"
	// R_12 = R_01' * R_02 = R_10 * R_02
	// R_12 = (eta_21, eps_1_21) = (eta_21, eps_2_21)

	Eigen::Matrix3d R_ed = R_e.transpose() * R_d;
	Eigen::Vector3d eps_e_de = Eigen::Quaterniond(R_ed).vec();
	Eigen::Vector3d eps_de = R_e * eps_e_de; // to base frame

	// error vector [6 x 1]
	Eigen::Vector6d x_de;
	x_de << p_de, eps_de;

	return x_de;
}

void
CartesianAdmittanceController::callback_command(const geometry_msgs::PoseStampedConstPtr& msg)
{
	// read new transformation
	geometry_msgs::Pose pose_ref = msg.get()->pose;
	Eigen::Isometry3d T_ref = Eigen::make_tf(pose_ref);
	tf::poseMsgToEigen(pose_ref, T_ref);

	// ensure proper orientation
	static Eigen::Quaterniond quat_ref_prev = Eigen::Quaterniond(T_ref.rotation());
	Eigen::Quaterniond quat_ref = Eigen::Quaterniond(T_ref.rotation());
	if (quat_ref_prev.coeffs().dot(quat_ref.coeffs()) < 0.0)
	{
		quat_ref.coeffs() = -quat_ref.coeffs();
		T_ref.linear() = quat_ref.toRotationMatrix(); // set rotation (linear = rotation)
		// T_ref = Eigen::Translation3d(T_ref.translation()) * Eigen::Isometry3d(quat_ref);
	}
	quat_ref_prev = quat_ref;

	// write pose
	buffer_pose_ref.writeFromNonRT(Pose{ T_ref.translation(), T_ref.rotation() });
}

} // namespace franka_controllers

