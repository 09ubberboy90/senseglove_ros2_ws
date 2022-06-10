// Copyright 2020 senseglove
// Copyright 2022 Florent AUDONNET
#include "senseglove_hardware_interface/senseglove_hardware_interface.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
// code is inspired by
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // create executor
    std::shared_ptr<rclcpp::Executor> e = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // create controller manager instance
    auto controller_manager = std::make_shared<controller_manager::ControllerManager>(e, "controller_manager");

    // control loop thread
    std::thread control_loop([controller_manager]()
                             {
    // use fixed time step
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

    while (rclcpp::ok()) {
      // ur client library is blocking and is the one that is controlling time step
      controller_manager->read();
      controller_manager->update(controller_manager->now(), dt);
      controller_manager->write();
    } });

    // spin the executor with controller manager node
    e->add_node(controller_manager);
    e->spin();

    // wait for control loop to finish
    control_loop.join();

    // shutdown
    rclcpp::shutdown();

    return 0;
}