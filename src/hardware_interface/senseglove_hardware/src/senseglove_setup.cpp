// Copyright 2020 SenseGlove
// Copyright 2022 Florent AUDONNET

#include "senseglove_hardware/senseglove_setup.h"
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace senseglove
{
    SenseGloveSetup::SenseGloveSetup(senseglove::SenseGloveRobot senseglove)
        : senseglove_(std::move(senseglove))
    {
    }

    void SenseGloveSetup::startCommunication(bool /*reset*/)
    {
        if (SGCore::DeviceList::SenseCommRunning())
        {
            RCLCPP_WARN(rclcpp::get_logger("senseglove_setup"), "Trying to start senseglove communication while it is already active.");
            return;
        }
    }

    void SenseGloveSetup::stopCommunication()
    {
        this->getSenseGloveRobot().stopActuating();
    }

    bool SenseGloveSetup::isCommunicationOperational()
    {
        if (SGCore::DeviceList::SenseCommRunning())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    SenseGloveRobot &SenseGloveSetup::getSenseGloveRobot()
    {
        if (!SGCore::DeviceList::SenseCommRunning())
        {
            RCLCPP_WARN(rclcpp::get_logger("senseglove_setup"), "Trying to access joints while communication is not operational. This "
                                                                "may lead to incorrect sensor data.");
        }
        return senseglove_;
    }

    SenseGloveSetup::~SenseGloveSetup()
    {
        stopCommunication();
    }

} // namespace senseglove
