#include <franka_controllers/joint_position_pd_gravity_controller.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.hpp>

// export controller
PLUGINLIB_EXPORT_CLASS(franka_controllers::JointPositionPDGravityController, controller_interface::ControllerBase)

namespace franka_controllers
{
    
bool
JointPositionPDGravityController::init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh)
{

    std::string arm_id;
    std::vector<std::string> joint_names;

    if (!nh.getParam("arm_id", arm_id))
    { 
        ROS_ERROR("franka_controller: Could not read parameter arm_id"); 
        return false; 
    }

    // instantiate the franka joint_names
    if (!nh.getParam("joint_names", joint_names) || joint_names.size() != 7)
    { 
        ROS_ERROR( "franka_controller: Invalid or no joint_names parameters provided, aborting " "controller init!"); 
        return false; 
    }
    
    auto *model_interface = hw->get<franka_hw::FrankaModelInterface>();

    // does the model_interface exist
    if (model_interface == nullptr)
    { 
        ROS_ERROR_STREAM("franka_controller: Error getting model interface from hardware"); 
        return false; 
    }

    try
    { 
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>( model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    { 
        ROS_ERROR_STREAM( "franka_controller: Exception getting model handle from interface: " << ex.what());
        return false;
    }

    auto *state_interface = hw->get<franka_hw::FrankaStateInterface>();

    // does the state_interface exist 
    if (state_interface == nullptr) 
    { 
        ROS_ERROR_STREAM("franka_controller: Error getting state interface from hardware"); 
        return false; 
    }
    try
    { 
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot")); 
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    { 
        ROS_ERROR_STREAM("franka_controller: Exception getting state handle from interface: " << ex.what()); 
        return false;
    }

    // does the effort_joint_interface exist
    auto *effort_joint_interface = hw->get<hardware_interface::EffortJointInterface>();

    if (effort_joint_interface == nullptr)
    { 
        ROS_ERROR_STREAM("franka_controller: Error getting effort joint interface from hardware");
        return false; 
    }

    // fill the joint_handles_
    for (size_t i = 0; i < num_joints; ++i)
    {
        try 
        { 
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException &ex) 
        { 
            ROS_ERROR_STREAM("franka_controller: Exception getting joint handles: " << ex.what()); 
            return false; 
        }
    }

    // initialize command buffer and q_d vector
    commands_buffer.writeFromNonRT( std::vector<double>(num_joints, 0.0) ); // 7d vector filled with 0.0

    q_d = Eigen::Vector7d::Zero();

    // subscribe to command "command" topic, such that we can control it from the terminal.
    sub_command = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointPositionPDGravityController::callback_command, this);

    std::cout << "sub_command.getTopic() : " << sub_command.getTopic() << std::endl;

    // init complete
    ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME << " with kp = " << kp << ", kd = " << kd);
    ROS_INFO_STREAM_NAMED(CONTROLLER_NAME," has been initialized successfully");

    return true;
}

void 
JointPositionPDGravityController::starting(const ros::Time &time)
{
    // initial desired position
    commands_buffer.writeFromNonRT(Q_D_INIT);
}

void 
JointPositionPDGravityController::update(const ros::Time &time, const ros::Duration &period)
{
    // elapsed time
    static ros::Duration elapsed_time = ros::Duration(0.);
    elapsed_time += period;

    // get desired joint efforts
    const std::vector<double> &commands = *commands_buffer.readFromRT();

    if (commands.empty())
        return;

    for (size_t i = 0; i < commands.size(); ++i)
        q_d[i] = commands[i];

    // read joint states
    const auto q    = get_position();
    const auto qdot = get_velocity();

    const auto g = get_gravity();

    // compute controller effort
    Eigen::Vector7d tau_des = kp * (q_d - q) - kd * qdot + g;

    // saturate rate-of-effort (rotatum)
    if (SATURATE_ROTATUM)
        tau_des = saturate_rotatum(tau_des, period.toSec());

    // set desired command on joint handles
    for (size_t i = 0; i < num_joints; ++i)
        joint_handles_[i].setCommand(tau_des[i]);
}

Eigen::Vector7d
JointPositionPDGravityController::get_position()
{
    std::array<double, 7> q_array = state_handle_->getRobotState().q;
    Eigen::Map<Eigen::Vector7d> Q(q_array.data());
    return Q;
}

Eigen::Vector7d
JointPositionPDGravityController::get_velocity()
{
    std::array<double, 7> dq_array = state_handle_->getRobotState().dq;
    Eigen::Map<Eigen::Vector7d> dQ(dq_array.data());
    return dQ;
}

Eigen::Vector7d
JointPositionPDGravityController::get_gravity()
{
    std::array<double, 7> gravity_array = model_handle_->getGravity();
    Eigen::Map<Eigen::Vector7d> g(gravity_array.data());
    return g;
}

Eigen::Matrix7x7
JointPositionPDGravityController::get_mass()
{
    std::array<double, 49> m_array = model_handle_->getMass();
    Eigen::Map<Eigen::Matrix7x7> m(m_array.data());
    return m;
}

Eigen::Matrix6x7
JointPositionPDGravityController::get_jacobian()
{
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix6x7> jacobian(jacobian_array.data());
    return jacobian;
}

Eigen::Vector7d
JointPositionPDGravityController::saturate_rotatum(const Eigen::Vector7d &tau_des, const double period)
{
    // previous desired torque and saturated torque
    static Eigen::Vector7d tau_des_prev = Eigen::Vector7d::Zero();
    static Eigen::Vector7d tau_des_sat = Eigen::Vector7d::Zero();

    // compute saturated torque
    for (size_t i = 0; i < tau_des_sat.size(); ++i)
    {
        const double tau_dot = (tau_des[i] - tau_des_prev[i]) / period;
        tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(tau_dot, TAU_DOT_MAX * period), -(TAU_DOT_MAX * period));
    }

    // save for next iteration and return
    tau_des_prev = tau_des_sat;
    return tau_des_sat;
}

void 
JointPositionPDGravityController::callback_command(const std_msgs::Float64MultiArrayConstPtr &msg)
{

    if (msg->data.empty())
        ROS_WARN("Data was empty %ld",msg->data.size());
    else
        ROS_WARN("Data size: %ld", msg->data.size());

    // check size
    if (msg->data.size() != num_joints)
    {
        ROS_ERROR_NAMED(CONTROLLER_NAME, "Number of desired values in command (%lu) does not match number of joints (%lu); execution aborted.", msg->data.size(), num_joints);
        return;
    }
    // write commands to command buffer
    commands_buffer.writeFromNonRT(msg->data);

    // for testing purposes
    // for (size_t i = 0; i < num_joints; ++i)
    //     joint_handles_[i].setCommand(msg->data[i]);
}

} // namespace franka_controllers

