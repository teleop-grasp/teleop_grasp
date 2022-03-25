#pragma once

#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <controller_interface/multi_interface_controller.h>

namespace Eigen
{
    using Vector7d  = Eigen::Matrix<double,7,1>;
    using Matrix7x7 = Eigen::Matrix<double,7,7>;
    using Matrix6x7 = Eigen::Matrix<double,6,7>;
}

namespace franka_controllers
{
    class JointPositionPDGravityController final : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                                                                         hardware_interface::EffortJointInterface,
                                                                                                         franka_hw::FrankaStateInterface>
    {
    public:
        static inline constexpr auto CONTROLLER_NAME     = "JointPositionPDGravityController";
        static inline constexpr auto SATURATE_ROTATUM    = true;
        static inline constexpr auto TAU_DOT_MAX         = 1000.;
        static inline const std::vector<double> Q_D_INIT = {0, 0, 0, -1.57079632679, 0, 1.57079632679, 0.785398163397}; // taken from panda.launch

        std::vector<std::string>
            joint_names;
        std::string              arm_id;
        size_t                   num_joints = 7;

        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;

        JointPositionPDGravityController() {}
        ~JointPositionPDGravityController() { sub_command.shutdown(); }

        bool 
        init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh) override;

        void 
        starting(const ros::Time &time) override;

        void 
        update(const ros::Time &time, const ros::Duration &period) override;

    private:

        ros::Subscriber sub_command;

        Eigen::Vector7d q_d;

        double kp = 200.0;
        double kd = 100.0;

        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        Eigen::Vector7d
        get_position();

        Eigen::Vector7d
        get_velocity();

        Eigen::Vector7d
        get_gravity();

        Eigen::Matrix7x7
        get_mass();

        Eigen::Matrix6x7
        get_jacobian();

        Eigen::Vector7d
        saturate_rotatum(const Eigen::Vector7d &tau_des, const double period = 0.001 /* [s] */);

        void
        callback_command(const std_msgs::Float64MultiArrayConstPtr &msg);
    };
}