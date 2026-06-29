/**
 * @brief Facade implementation for planning pipeline services.
 *
 * Each public method is responsible for only two things:
 *   1. Build the service request.
 *   2. Map the service response to the return value.
 *
 * The entire async + future.wait_for + timeout pattern is handled
 * once in Planning::call_service<T>() (defined in the header).
 *
 * File I/O is isolated in load_text_file() and is_txt_file() so
 * parseSemanticCommand() stays focused on service communication.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "planning_tools/Planning.hpp"
#include "semantic_parser_interfaces/srv/extract_entity.hpp"
#include <chrono>
#include <memory>
#include <vector>

// ============================================================
// Constructor
// ============================================================

Planning::Planning(const rclcpp::Node::SharedPtr &node) : node_(node) {
  cb_group_clients_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Guys, please. Make sure the code compiles with ROS 2 Humble too. We talked
  // about this!!!
  // const rclcpp::QoS qos = rclcpp::ServicesQoS();
  const rmw_qos_profile_t qos = rmw_qos_profile_services_default;

  interpret_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/hri/listener/natural_to_clips/interpret", qos, cb_group_clients_);

  primitive_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/planning/primitive_to_clips/convert", qos, cb_group_clients_);

  goals_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/planning/goals_to_clips/convert", qos, cb_group_clients_);

  entity_client_ =
      node->create_client<semantic_parser_interfaces::srv::ExtractEntity>(
          "/planning/semantic_parser/extract_entity", qos, cb_group_clients_);

  request_instruction_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/planning/request_instruction", qos, cb_group_clients_);

  clips_start_planning_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/planning/clips/start_planning", qos, cb_group_clients_);

  semantic_parser_client_ =
      node_->create_client<semantic_parser_interfaces::srv::Parser>(
          "/planning/semantic_parser/semantic_parser", qos, cb_group_clients_);
}

// ============================================================
// interpretToClips
// ============================================================

std::string Planning::interpretToClips(const std::string &sentence,
                                       std::chrono::seconds timeout) {
  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = sentence;

  std::string result;

  call_service<clips_interfaces::srv::String>(
      interpret_client_, request,
      [&](auto response) { result = response->data; }, timeout,
      "InterpretToClips");

  return result;
}

// ============================================================
// primitiveToClips
// ============================================================

std::string Planning::primitiveToClips(const std::string &sentence,
                                       std::chrono::seconds timeout) {
  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = sentence;

  std::string result;

  call_service<clips_interfaces::srv::String>(
      primitive_client_, request,
      [&](auto response) { result = response->data; }, timeout,
      "PrimitiveToClips");

  return result;
}

// ============================================================
// goalsToClips
// ============================================================

std::string Planning::goalsToClips(const std::string &goals_json,
                                   std::chrono::seconds timeout) {
  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = goals_json;

  std::string result;

  call_service<clips_interfaces::srv::String>(
      goals_client_, request,
      [&](auto response) { result = response->data; }, timeout,
      "GoalsToClips");

  return result;
}

// ============================================================
// extractEntity
// ============================================================

std::vector<std::string> Planning::extractEntity(const std::string &sentence,
                                                 const std::string &category,
                                                 std::chrono::seconds timeout) {

  auto request = std::make_shared<
      semantic_parser_interfaces::srv::ExtractEntity::Request>();

  request->sentence = sentence;
  request->category = category;

  std::vector<std::string> result;

  bool completed = call_service<semantic_parser_interfaces::srv::ExtractEntity>(
      entity_client_, request,
      [&](auto response) {
        if (!response->value.empty()) {
          result = {response->value};
        } else if (!response->values.empty()) {
          result = response->values;
        }
      },
      timeout, "ExtractEntity");

  if (!completed) {
    return {};
  }
  if (result.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "ExtractEntity returned empty result for category '%s'.",
                category.c_str());
  }

  return result;
}

// ============================================================
// requestInstruction
// ============================================================

std::vector<std::string>
Planning::requestInstruction(std::chrono::seconds timeout) {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  while (rclcpp::ok()) {
    std::string message;
    bool success = false;

    bool completed = call_service<std_srvs::srv::Trigger>(
        request_instruction_client_, request,
        [&](auto response) {
          success = response->success;
          message = response->message;
        },
        timeout, "RequestInstruction");

    if (completed && success && !message.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Instruction received: %s",
                  message.c_str());
      return tokenize(message);
    }

    RCLCPP_WARN(node_->get_logger(), "Waiting for instruction...");
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }

  return {};
}

// ============================================================
// startPlanning
// ============================================================

bool Planning::startPlanning(std::chrono::seconds timeout) {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  bool success = false;

  call_service<std_srvs::srv::Trigger>(
      clips_start_planning_client_, request,
      [&](auto response) { success = response->success; }, timeout,
      "StartPlanning");

  return success;
}

// ============================================================
// parseSemanticCommand
// ============================================================

std::string Planning::parseSemanticCommand(const std::string &input,
                                           std::chrono::seconds timeout) {
  const std::string command =
      is_txt_file(input) ? load_text_file(input) : input;

  if (command.empty()) {
    return "";
  }

  auto request =
      std::make_shared<semantic_parser_interfaces::srv::Parser::Request>();
  request->command = command;

  std::string result;

  bool completed = call_service<semantic_parser_interfaces::srv::Parser>(
      semantic_parser_client_, request,
      [&](auto response) { result = response->response; }, timeout,
      "SemanticParser");

  if (!completed) {
    return "";
  }

  if (result.empty()) {
    RCLCPP_WARN(node_->get_logger(), "SemanticParser returned empty response.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Semantic CD: %s", result.c_str());
  }

  return result;
}

// ============================================================
// Private helpers
// ============================================================

std::vector<std::string> Planning::tokenize(const std::string &input) const {
  std::vector<std::string> tokens;
  std::istringstream stream(input);
  std::string token;

  while (stream >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

std::string Planning::load_text_file(const std::string &path) const {
  std::ifstream file(path);

  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not open file: %s", path.c_str());
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();

  RCLCPP_INFO(node_->get_logger(), "Loaded command from file: %s",
              path.c_str());

  return buffer.str();
}

bool Planning::is_txt_file(const std::string &input) const {
  return std::filesystem::exists(input) &&
         std::filesystem::is_regular_file(input) && input.size() >= 4 &&
         input.substr(input.size() - 4) == ".txt";
}
