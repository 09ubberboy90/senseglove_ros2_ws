// Copyright 2020 senseglove
// Copyright 2022 Florent AUDONNET
#ifndef ROS_WORKSPACE_SENSEGLOVE_HARDWARE_INTERFACE_H
#define ROS_WORKSPACE_SENSEGLOVE_HARDWARE_INTERFACE_H

#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/visibility_control.h>

#include <senseglove_hardware/senseglove_robot.h>
#include <senseglove_hardware/senseglove_setup.h>

#include <senseglove_hardware_builder/hardware_builder.h>
#include <senseglove_shared_resources/msg/sense_glove_state.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

/**
 * @brief HardwareInterface to allow rclcpp_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via rclcpp_control.
 */
class SenseGloveHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // RCLCPP_SHARED_PTR_DEFINITIONS(SenseGloveHardwareInterface);
    /**
     * @brief Initialize the HardwareInterface by registering position interfaces
     * for each joint.
     */
    CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) final;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                                const std::vector<std::string> &stop_interfaces) final;

    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                                const std::vector<std::string> &stop_interfaces) final;
    hardware_interface::return_type read() final;
    hardware_interface::return_type write() final;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

private:
    void reserveMemory();

    /* SenseGlove hardware */
    std::unique_ptr<senseglove::SenseGloveSetup> senseglove_setup_;
    /* Shared memory */
    size_t num_gloves_ = 0;
    size_t num_joints_ = 0;
    size_t num_sensors_ = 0;

    std::vector<double> sensor_states_;

    std::vector<double> joint_position_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_last_position_command_;

    std::vector<double> joint_velocity_;
    std::vector<double> joint_velocity_command_;
    
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_last_effort_command_;
    std::vector<double> joint_last_buzz_command_; // inherited from effort_command

    bool has_actuated_ = false;
    bool controllers_initialized_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    std::vector<std::string> start_modes_;
};

#endif // ROS_WORKSPACE_SG_HARDWARE_INTERFACE_H
