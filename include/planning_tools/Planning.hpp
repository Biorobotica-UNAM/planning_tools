#pragma once

#include "clips_interfaces/srv/string.hpp"
#include "semantic_parser_interfaces/srv/parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>

class Planning {
public:
  explicit Planning(const rclcpp::Node::SharedPtr &node);

  std::string interpretToClips(const std::string &sentence);
  std::string primitiveToClips(const std::string &sentence);
  std::vector<std::string> requestInstruction();
  bool startPlanning();

  std::string parseSemanticCommand(const std::string &command);

private:
  bool waitForService(const rclcpp::ClientBase::SharedPtr &client,
                      const std::chrono::seconds &timeout) const;

  std::vector<std::string> tokenize(const std::string &input);

  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

  rclcpp::Client<clips_interfaces::srv::String>::SharedPtr interpret_client_;
  rclcpp::Client<clips_interfaces::srv::String>::SharedPtr primitive_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_instruction_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clips_start_planning_client_;

  rclcpp::Client<semantic_parser_interfaces::srv::Parser>::SharedPtr semantic_parser_client_;
};
