#include <franka_controllers/joint_position_pd_gravity_controller.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.hpp>

// #include <ros_utils/ros.h>

// export controller
PLUGINLIB_EXPORT_CLASS(franka_controllers::JointPositionPDGravityController, controller_interface::ControllerBase)

namespace franka_controllers
{

bool
JointPositionPDGravityController::init(hardware_interface::RobotHW *hw, ros::NodeHandle& nh)
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

	// read Kp gains from config
	if (std::vector<double> vec_kp; nh.getParam("kp", vec_kp) and vec_kp.size() == num_joints)
	{
		Kp = Eigen::Vector7d(vec_kp.data()).asDiagonal();
	}
	else
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read Kp gain");
		return false;
	}
	
	// read kd gains from config
	if (std::vector<double> vec_kd; nh.getParam("kd", vec_kd) and vec_kd.size() == num_joints)
	{
		Kd = Eigen::Vector7d(vec_kd.data()).asDiagonal();
	}
	else
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read Kd gain");
		return false;
	}
	
	// read dtau_max from config (-1 to disable)
	if (not nh.getParam("dtau_max", dtau_max))
	{
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read dtau_max ");
		return false;
	}
	
	// read initial (home) joint position from config
	if (std::vector<double> vec_q_d; nh.getParam("q_d", vec_q_d) and vec_q_d.size() == num_joints)
	{
		q_d = Eigen::Vector7d(vec_q_d.data());
	}
	else
	{ 
		ROS_ERROR_STREAM(CONTROLLER_NAME << ": Could not read q_d");
		return false;
	}

	// subscribe to command topic
	sub_command = nh.subscribe<std_msgs::Float64MultiArray>("command", 1,& JointPositionPDGravityController::callback_command, this);

	// init complete
	ROS_INFO_STREAM("Initialized " << CONTROLLER_NAME << " with: "
		<< "Kp = "       << Kp.diagonal().transpose() << ", "
		<< "Kd = "       << Kd.diagonal().transpose() << ", "
		<< "q_d = "      << q_d.transpose()  << ", "
		<< "dtau_max = " << dtau_max
	);

	return true;
}

void
JointPositionPDGravityController::starting(const ros::Time& time)
{
	// initial desired position
	std::vector<double> q_d_array(q_d.data(), q_d.data() + q_d.size()); // convert Eigen to std::vector
	commands_buffer.writeFromNonRT(q_d_array);
}

void
JointPositionPDGravityController::update(const ros::Time& time, const ros::Duration& period)
{
	// elapsed time
	static ros::Duration elapsed_time = ros::Duration(0.);
	elapsed_time += period;

	// get desired joint efforts
	const std::vector<double>& commands = *commands_buffer.readFromRT();
	for (size_t i = 0; i < commands.size(); ++i)
		q_d[i] = commands[i];

	// read robot state
	const auto q  = get_position();
	const auto dq = get_velocity();
	const auto g  = get_gravity();

	// compute controller effort
	Eigen::Vector7d tau_d = Kp * (q_d - q) - Kd * dq + g;

	// saturate rate-of-effort (rotatum)
	if (dtau_max > 0)
		tau_d = saturate_rotatum(tau_d, period.toSec());

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		joint_handles[i].setCommand(tau_d[i]);
}

Eigen::Vector7d
JointPositionPDGravityController::get_position()
{
	auto q_array = state_handle->getRobotState().q;
	return Eigen::Vector7d(q_array.data());
}

Eigen::Vector7d
JointPositionPDGravityController::get_velocity()
{
	auto dq_array = state_handle->getRobotState().dq;
	return Eigen::Vector7d(dq_array.data());
}

Eigen::Vector7d
JointPositionPDGravityController::get_gravity()
{
	auto g_array = model_handle->getGravity();
	return Eigen::Vector7d(g_array.data());
}

Eigen::Vector7d
JointPositionPDGravityController::saturate_rotatum(const Eigen::Vector7d& tau_d, const double period)
{
	// previous desired torque and saturated torque
	static Eigen::Vector7d tau_d_prev = Eigen::Vector7d::Zero();
	static Eigen::Vector7d tau_d_sat = Eigen::Vector7d::Zero();

	// compute saturated torque
	for (size_t i = 0; i < tau_d_sat.size(); ++i)
	{
		const double dtau = (tau_d[i] - tau_d_prev[i]) / period;
		tau_d_sat[i] = tau_d_prev[i] + std::max(std::min(dtau, dtau_max * period), -(dtau_max * period));
	}

	// save for next iteration and return
	tau_d_prev = tau_d_sat;
	return tau_d_sat;
}

void
JointPositionPDGravityController::callback_command(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	// check size
	if (msg->data.size() != num_joints)
	{
		ROS_ERROR_NAMED(CONTROLLER_NAME, "Number of desired values in command (%lu) does not match number of joints (%lu); execution aborted.", msg->data.size(), num_joints);
		return;
	}

	// write commands to command buffer
	commands_buffer.writeFromNonRT(msg->data);
}

} // namespace franka_controllers

