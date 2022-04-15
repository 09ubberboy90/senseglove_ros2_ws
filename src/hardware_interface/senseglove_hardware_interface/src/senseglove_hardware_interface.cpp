// Copyright 2020 SenseGlove
// Copyright 2022 Florent AUDONNET
#include "senseglove_hardware_interface/senseglove_hardware_interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <string>

#include <urdf/model.h>

bool to_bool(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}

CallbackReturn SenseGloveHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    std::string robot_type = info_.hardware_parameters["robot_type"];
    std::string hand_type = info_.hardware_parameters["is_right"];
    AllowedRobot robot = AllowedRobot(robot_type);
    HardwareBuilder builder = HardwareBuilder(robot, to_bool(hand_type));
    try
    {
        senseglove_setup_ = builder.createSenseGloveSetup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),"Hardware interface caught an exception during building hardware");
        RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),"%s", e.what());
        return CallbackReturn::FAILURE;
    }
    // Start ethercat cycle in the hardware
    this->senseglove_setup_->startCommunication(true);
    return CallbackReturn::SUCCESS;
}
CallbackReturn SenseGloveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    // Start ethercat cycle in the hardware
    this->senseglove_setup_->stopCommunication();
    return CallbackReturn::SUCCESS;
}

CallbackReturn SenseGloveHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    info_ = hardware_info;
    num_joints_ = info_.joints.size();
    // initialize
    this->reserveMemory();

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),
                         "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),
                         "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),
                         "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),
                         "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),
                         "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SenseGloveHardwareInterface"),
                         "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
    ;
}

std::vector<hardware_interface::StateInterface> SenseGloveHardwareInterface::export_state_interfaces()
{

    std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_effort_[i]));
        }
    return state_interfaces;
}

// Documentation Inherited
std::vector<hardware_interface::CommandInterface> SenseGloveHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]));
        }
    return command_interfaces;
}

hardware_interface::return_type SenseGloveHardwareInterface::read()
{
    if (senseglove_setup_->getSenseGloveRobot().updateGloveData())
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            senseglove::Joint &joint = senseglove_setup_->getSenseGloveRobot().getJoint(info_.joints[i].name);

            joint_position_[i] = joint.getPosition();
            joint_velocity_[i] = joint.getVelocity();
            joint_effort_[i] = joint.getTorque();
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SenseGloveHardwareInterface::write()
{
    // Splice joint_effort_command vector into vectors for ffb and buzz commands
    int j = 0;
    int h = 0;
    senseglove::SenseGloveRobot &robot = senseglove_setup_->getSenseGloveRobot();
    for (size_t i = 0; i < num_joints_; i++)
    {
        senseglove::Joint &joint = robot.getJoint(info_.joints[i].name);
        if (joint.canActuate())
        {
            if (joint.getName() == "pinky_brake")
            {
                // RCLCPP_INFO_STREAM(rclcpp::get_logger("Writings"), "" << joint.getName() << ": " << joint.getPosition() << joint_position_command_[i]);
            }
            
            if (joint.getActuationMode() == senseglove::ActuationMode::position)
            {
                if (j % 2 == 1)
                {
                    joint_last_position_command_[h] = joint_position_command_[i];
                    h++;
                }
                else
                {
                    joint_last_buzz_command_[h] = joint_position_command_[i];
                }

                if (j == 9) // actuators 0 - 9
                {
                    robot.actuateEffort(joint_last_position_command_);
                    robot.actuateBuzz(joint_last_buzz_command_);
                }
            }
            else if (joint.getActuationMode() == senseglove::ActuationMode::torque)
            {
                if (j % 2 == 1)
                {
                    joint_last_effort_command_[j] = joint_effort_command_[i];
                    h++;
                }
                else
                {
                    joint_last_buzz_command_[j] = joint_effort_command_[i];
                }

                if (j == 9) // actuators 0 - 9
                {
                    robot.actuateEffort(joint_last_effort_command_);
                    robot.actuateBuzz(joint_last_buzz_command_);
                }
            }
            j++;
        }
    }
    j = 0;
    h = 0;
    return hardware_interface::return_type::OK;
}

void SenseGloveHardwareInterface::reserveMemory()
{
    joint_position_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_last_position_command_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);
    joint_last_effort_command_.resize(5, 0.0);
    joint_last_buzz_command_.resize(5, 0.0);

}

hardware_interface::return_type SenseGloveHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    start_modes_.clear();

    // Starting interfaces
    // add start interface per joint in tmp var for later check
    for (const auto &key : start_interfaces)
    {
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
            {
                start_modes_.push_back(hardware_interface::HW_IF_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
            {
                start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
            }
        }
    }
    // all start interfaces must be the same - can't mix position and velocity control
    if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin()))
    {
        ret_val = hardware_interface::return_type::ERROR;
    }

    controllers_initialized_ = true;
    return ret_val;
}

hardware_interface::return_type SenseGloveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    if (start_modes_.size() != 0 &&
        std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end())
    {
        velocity_controller_running_ = false;
        joint_position_command_ = joint_last_position_command_ = joint_position_;
        position_controller_running_ = true;
    }
    else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(),
                                                   hardware_interface::HW_IF_VELOCITY) != start_modes_.end())
    {
        position_controller_running_ = false;
        std::fill(joint_velocity_command_.begin(), joint_velocity_command_.end(), 0);

        velocity_controller_running_ = true;
    }

    start_modes_.clear();

    return ret_val;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(SenseGloveHardwareInterface, hardware_interface::SystemInterface)