#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "clips_interfaces/srv/string.hpp"

#include <string>
#include <memory>
#include <chrono>

class Planning
{
public:
    Planning(const rclcpp::Node::SharedPtr& node);

    // Convierte lenguaje natural a una representación CLIPS
    std::string interpretToClips(const std::string& sentence);

    // Solicita una instrucción (Trigger)
    bool requestInstruction();

private:
    bool waitForService(
        const rclcpp::ClientBase::SharedPtr& client,
        const std::string& service_name,
        std::chrono::seconds timeout);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<clips_interfaces::srv::String>::SharedPtr interpret_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_instruction_client_;
};

