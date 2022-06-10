// Copyright 2020 Senseglove
// Copyright 2022 Florent AUDONNET

#include "senseglove_hardware_builder/hardware_builder.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "SenseGlove.h"
#include <SGConnect.h>

const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = {"allowActuation", "jointIndex", "minPosition",
                                                                       "maxPosition"};
const std::vector<std::string> HardwareBuilder::ROBOT_REQUIRED_KEYS = {"deviceType"};

HardwareBuilder::HardwareBuilder(AllowedRobot robot, bool is_right)
    : HardwareBuilder(robot.getFilePath(), is_right)
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot)
    : robot_config_(YAML::LoadFile(robot.getFilePath()))
{
}

HardwareBuilder::HardwareBuilder(const std::string &yaml_path, bool is_right)
    : robot_config_(YAML::LoadFile(yaml_path)), is_right_(is_right)
{
}

HardwareBuilder::HardwareBuilder(const std::string &yaml_path)
    : robot_config_(YAML::LoadFile(yaml_path))
{
}

std::unique_ptr<senseglove::SenseGloveSetup> HardwareBuilder::createSenseGloveSetup()
{
    const auto robot_name = this->robot_config_.begin()->first.as<std::string>();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hardware_builder"), "Starting creation of robot " << robot_name);

    // Remove top level robot name key
    YAML::Node config = this->robot_config_[robot_name];

    std::vector<SGCore::SG::SenseGlove> all_gloves = SGCore::SG::SenseGlove::GetSenseGloves();
    SGCore::SG::SenseGlove current_glove; // Will be update later
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hardware_builder"), "creating sensegloves");

    if (SGCore::DeviceList::SenseCommRunning())
    {
        RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "Obtained the following gloves: ");
        for (auto &all_glove : all_gloves)
        {
            RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "%s", all_glove.ToString().c_str());
        }
        current_glove = correct_glove(all_gloves);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hardware_builder"), "Obtained devicetype from glove");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware_builder"), "No Sensegloves connected");
        std::exit(1);
    }

    std::vector<senseglove::Joint> joints = this->createJoints(config["joints"]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hardware_builder"), "Created joints " << joints.size());
    senseglove::SenseGloveRobot sensegloves =
        HardwareBuilder::createRobot(config, std::move(joints), current_glove, is_right_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hardware_builder"), "Created Robots " << sensegloves.getName() << ", is right: "
                                                                                 << sensegloves.getRight() << " is urdfright: " << current_glove.IsRight());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hardware_builder"), "Robot config:\n"
                                                                   << config);
    return std::make_unique<senseglove::SenseGloveSetup>(std::move(sensegloves));
}

senseglove::Joint HardwareBuilder::createJoint(const YAML::Node &joint_config, const std::string &joint_name)
{
    RCLCPP_DEBUG(rclcpp::get_logger("hardware_builder"), "Starting creation of joint %s", joint_name.c_str());
    HardwareBuilder::validateRequiredKeysExist(joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint");

    auto joint_index = -1;
    if (joint_config["jointIndex"])
    {
        joint_index = joint_config["jointIndex"].as<int>();
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("hardware_builder"), "Joint %s does not have a netNumber", joint_name.c_str());
    }

    const auto allow_actuation = joint_config["allowActuation"].as<bool>();

    senseglove::ActuationMode mode;
    if (joint_config["actuationMode"])
    {
        mode = senseglove::ActuationMode(joint_config["actuationMode"].as<std::string>());
    }

    return {joint_name, joint_index, allow_actuation, mode};
}

senseglove::SenseGloveRobot HardwareBuilder::createRobot(const YAML::Node &robot_config,
                                                         std::vector<senseglove::Joint> jointList,
                                                         SGCore::SG::SenseGlove glove,
                                                         bool is_arg_right)
{
    RCLCPP_DEBUG(rclcpp::get_logger("hardware_builder"), "Starting creation of glove %s", glove.ToString().c_str());
    HardwareBuilder::validateRequiredKeysExist(robot_config, HardwareBuilder::ROBOT_REQUIRED_KEYS, "glove");
    bool is_glove_right = glove.IsRight();

    if (is_glove_right xor is_arg_right)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware_builder"), "IsRight parameters does not match found glove!\n %d, %d",
                     is_glove_right, is_arg_right);
        std::exit(1);
    }

    return {glove, std::move(jointList), is_glove_right};
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node &config, const std::vector<std::string> &key_list,
                                                const std::string & /*object_name*/)
{
    for (const std::string &key : key_list)
    {
        if (!config[key])
        {
            RCLCPP_ERROR(rclcpp::get_logger("hardware_builder"), "Missing Key");
            //            throw MissingKeyException(key, object_name);
        }
    }
}

std::vector<senseglove::Joint> HardwareBuilder::createJoints(const YAML::Node &joints_config) const
{
    std::vector<senseglove::Joint> joints;
    for (const YAML::Node &joint_config : joints_config)
    {
        const auto joint_name = joint_config.begin()->first.as<std::string>();
        joints.push_back(HardwareBuilder::createJoint(joint_config[joint_name], joint_name));
    }

    joints.shrink_to_fit();
    return joints;
}

std::vector<senseglove::SenseGloveRobot> HardwareBuilder::createRobots(
    const YAML::Node &robots_config, std::vector<senseglove::Joint> jointList,
    std::vector<SGCore::SG::SenseGlove> all_gloves) const
{
    std::vector<senseglove::SenseGloveRobot> robots;

    int i = 0;
    for (auto &glove : all_gloves)
    {
        robots.push_back(HardwareBuilder::createRobot(robots_config, std::move(jointList), glove, true)); // dubious fix
        i++;
    }

    robots.shrink_to_fit();
    return robots;
}

SGCore::SG::SenseGlove HardwareBuilder::correct_glove(std::vector<SGCore::SG::SenseGlove> gloves) const
{
    for (auto &glove :gloves)
    {
        if (glove.IsRight() && is_right_)
        {
            return glove;
        }
        if (!glove.IsRight() && !is_right_)
        {
            return glove;
        }
    }
    // Should never happen
    RCLCPP_WARN(rclcpp::get_logger("hardware_builder"), "Returning first glove as none matching the requirement could be found");
    return gloves[0];
}
