// Copyright 2020 SenseGlove.
#ifndef ROS_WORKSPACE_SENSEGLOVE_SETUP_H
#define ROS_WORKSPACE_SENSEGLOVE_SETUP_H

#include "SenseGlove.h"
#include "senseglove_hardware/joint.h"
#include "senseglove_hardware/senseglove_robot.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace senseglove
{
    class SenseGloveSetup
    {
    private:
        ::senseglove::SenseGloveRobot senseglove_;

    public:
        using iterator = std::vector<senseglove::SenseGloveRobot>::iterator;

        SenseGloveSetup(senseglove::SenseGloveRobot sensegloves);

        SenseGloveSetup(std::vector<senseglove::SenseGloveRobot> sensegloves);

        ~SenseGloveSetup();

        /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
        SenseGloveSetup(SenseGloveSetup &) = delete;
        SenseGloveSetup &operator=(SenseGloveSetup &) = delete;

        /* Delete move assignment since string cannot be move assigned */
        SenseGloveSetup(SenseGloveSetup &&) = delete;
        SenseGloveSetup &operator=(SenseGloveSetup &&) = delete;

        void startCommunication(bool /*reset*/);

        void stopCommunication();

        bool isCommunicationOperational();

        SenseGloveRobot &getSenseGloveRobot();

        size_t size() const;

        iterator begin();
        iterator end();

    };
} // namespace senseglove

#endif // ROS_WORKSPACE_SENSEGLOVE_SETUP_H
