#include "planning_tools/Planning.hpp"

#include <sstream>
#include <future>

using namespace std::chrono_literals;

Planning::Planning(const rclcpp::Node::SharedPtr &node) : node_(node)
{
  // callback group separado para clients
  cb_group_clients_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  interpret_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/hri/listener/natural_to_clips/interpret",
      rmw_qos_profile_services_default,
      cb_group_clients_);

  primitive_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/planning/primitive_to_clips/convert",
      rmw_qos_profile_services_default,
      cb_group_clients_);

  request_instruction_client_ =
      node_->create_client<std_srvs::srv::Trigger>(
          "/planning/request_instruction",
          rmw_qos_profile_services_default,
          cb_group_clients_);

  clips_start_planning_client_ =
      node_->create_client<std_srvs::srv::Trigger>(
          "/planning/clips/start_planning",
          rmw_qos_profile_services_default,
          cb_group_clients_);

  semantic_parser_client_ =
      node_->create_client<semantic_parser_interfaces::srv::Parser>(
          "/planning/semantic_parser/semantic_parser",
          rmw_qos_profile_services_default,
          cb_group_clients_);
}


bool Planning::waitForService(const rclcpp::ClientBase::SharedPtr &client,
                              const std::chrono::seconds &timeout) const
{
  return client->wait_for_service(timeout);
}

std::string Planning::interpretToClips(const std::string &sentence)
{
  if (!waitForService(interpret_client_, 5s)) {
    RCLCPP_ERROR(node_->get_logger(), "interpret service not available");
    return "";
  }

  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = sentence;

  auto future = interpret_client_->async_send_request(request);

  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling interpret service");
    return "";
  }

  auto response = future.get();
  return response->data;
}

std::string Planning::primitiveToClips(const std::string &sentence)
{
  if (!waitForService(primitive_client_, 5s)) {
    RCLCPP_ERROR(node_->get_logger(), "primitive_to_clips service not available");
    return "";
  }

  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = sentence;

  auto future = primitive_client_->async_send_request(request);

  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling primitive_to_clips service");
    return "";
  }

  auto response = future.get();
  return response->data;
}

std::vector<std::string> Planning::requestInstruction()
{
  if (!waitForService(request_instruction_client_, 5s)) {
    RCLCPP_ERROR(node_->get_logger(), "request_instruction service not available");
    return {};
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  while (rclcpp::ok()) {
    auto future = request_instruction_client_->async_send_request(request);

    if (future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(node_->get_logger(), "Waiting for instruction...");
      rclcpp::sleep_for(300ms);
      continue;
    }

    auto response = future.get();

    if (response->success && !response->message.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Instruction received: %s", response->message.c_str());
      return tokenize(response->message);
    }

    rclcpp::sleep_for(300ms);
  }

  return {};
}

std::vector<std::string> Planning::tokenize(const std::string &input)
{
  std::vector<std::string> tokens;
  std::istringstream iss(input);
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

bool Planning::startPlanning()
{
  if (!waitForService(clips_start_planning_client_, 5s)) {
    RCLCPP_ERROR(node_->get_logger(), "clips_start_planning service not available");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future  = clips_start_planning_client_->async_send_request(request);

  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Clips start planning timeout");
    return false;
  }

  auto response = future.get();
  return response->success;
}

std::string Planning::parseSemanticCommand(const std::string &input)
{
  std::string command = input;

  // If input is a .txt file path, load its content
  if (std::filesystem::exists(input) &&
      std::filesystem::is_regular_file(input) &&
      input.size() >= 4 &&
      input.substr(input.size() - 4) == ".txt")
  {
    std::ifstream file(input);
    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Could not open file: %s", input.c_str());
      return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    command = buffer.str();

    RCLCPP_INFO(node_->get_logger(), "Loaded command from file: %s", input.c_str());
  }

  if (!waitForService(semantic_parser_client_, 5s)) {
    RCLCPP_ERROR(node_->get_logger(), "semantic_parser service not available");
    return "";
  }

  auto request = std::make_shared<semantic_parser_interfaces::srv::Parser::Request>();
  request->command = command;

  auto future = semantic_parser_client_->async_send_request(request);

  // ✅ No spinning here. Let the MultiThreadedExecutor process responses.
  if (future.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling semantic_parser service");
    return "";
  }

  auto response = future.get();

  if (response->response.empty()) {
    RCLCPP_WARN(node_->get_logger(), "semantic_parser returned empty response");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Semantic CD: %s", response->response.c_str());
  }

  return response->response;
}
