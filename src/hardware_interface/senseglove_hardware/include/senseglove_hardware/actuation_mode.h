// Copyright 2019 Project March.

#ifndef SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
#define SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace senseglove
{
    class ActuationMode
    {
    public:
        enum Value : int
        {
            position,
            torque,
            unknown,
        };

        ActuationMode() : value_(unknown)
        {
        }

        ActuationMode(Value value) : value_(value)
        {
        }

        explicit ActuationMode(const std::string &actuationMode)
        {
            if (actuationMode == "position")
            {
                this->value_ = position;
            }
            else if (actuationMode == "unknown")
            {
                this->value_ = unknown;
            }
            else if (actuationMode == "torque")
            {
                this->value_ = torque;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("simple_moveit"), "Actuation mode (%s) is not recognized, setting to unknown mode", actuationMode.c_str());
                this->value_ = ActuationMode::unknown;
            }
        }

        uint8_t toModeNumber()
        {
            switch (this->value_)
            {
            case position:
                return 1;
            case torque:
                return 2;
            default:
                return 0;
            }
        }

        int getValue() const
        {
            return this->value_;
        }

        bool operator==(ActuationMode::Value a) const
        {
            return this->value_ == a;
        }

        bool operator!=(ActuationMode::Value a) const
        {
            return this->value_ != a;
        }

        std::string toString() const
        {
            switch (this->value_)
            {
            case position:
                return "position";
            case torque:
                return "torque";
            default:
                RCLCPP_WARN(rclcpp::get_logger("simple_moveit"), "Actuationmode (%i) is neither 'torque' or 'position", this->value_);
                return "unknown";
            }
        }

    private:
        Value value_ = unknown;
    };
} // namespace senseglove

#endif // SENSEGLOVE_HARDWARE_ACTUATION_MODE_H
