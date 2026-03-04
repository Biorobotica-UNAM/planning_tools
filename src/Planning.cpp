#include "planning_tools/Planning.hpp"

using namespace std::chrono_literals;

Planning::Planning(const rclcpp::Node::SharedPtr &node) : node_(node) {
  interpret_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/hri/listener/natural_to_clips/interpret");

  primitive_client_ = node_->create_client<clips_interfaces::srv::String>(
      "/planning/primitive_to_clips/convert");

  request_instruction_client_ =
      node_->create_client<std_srvs::srv::Trigger>("/planning/request_instruction");

  clips_start_planning_client_ =
      node_->create_client<std_srvs::srv::Trigger>("/planning/clips/start_planning");

  semantic_parser_client_ =
      node_->create_client<semantic_parser_interfaces::srv::Parser>(
          "/planning/semantic_parser/semantic_parser");
}

bool Planning::waitForService(const rclcpp::ClientBase::SharedPtr &client,
                              const std::chrono::seconds &timeout) const {
  return client->wait_for_service(timeout);
}

std::string Planning::interpretToClips(const std::string &sentence) {
  if (!waitForService(interpret_client_, 5s)) {
    return "";
  }

  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = sentence;

  std::mutex mtx;
  std::condition_variable cv;
  bool done = false;
  std::string result;

  auto callback =
      [&](rclcpp::Client<clips_interfaces::srv::String>::SharedFuture future) {
        result = future.get()->data;
        done = true;
        cv.notify_one();
      };

  interpret_client_->async_send_request(request, callback);

  std::unique_lock<std::mutex> lock(mtx);
  if (!cv.wait_for(lock, 5s, [&] { return done; })) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling interpret service");
    return "";
  }

  return result;
}


std::string Planning::primitiveToClips(const std::string &sentence) {
  if (!waitForService(primitive_client_, 5s)) {
    return "";
  }

  auto request = std::make_shared<clips_interfaces::srv::String::Request>();
  request->data = sentence;

  std::mutex mtx;
  std::condition_variable cv;
  bool done = false;
  std::string result;

  auto callback =
      [&](rclcpp::Client<clips_interfaces::srv::String>::SharedFuture future) {
        result = future.get()->data;
        done = true;
        cv.notify_one();
      };

  primitive_client_->async_send_request(request, callback);

  std::unique_lock<std::mutex> lock(mtx);
  if (!cv.wait_for(lock, 5s, [&] { return done; })) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling primitive service");
    return "";
  }

  return result;
}


std::vector<std::string> Planning::requestInstruction() {
  if (!waitForService(request_instruction_client_, 5s)) {
    return {};
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  while (rclcpp::ok()) {

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;
    std::string message;

    auto callback =
        [&](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          auto response = future.get();
          success = response->success;
          message = response->message;
          done = true;
          cv.notify_one();
        };

    request_instruction_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 5s, [&] { return done; })) {
      RCLCPP_WARN(node_->get_logger(), "Waiting for instruction...");
      rclcpp::sleep_for(300ms);
      continue;
    }

    if (success && !message.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Instruction received: %s",
                  message.c_str());
      return tokenize(message);
    }

    rclcpp::sleep_for(300ms);
  }

  return {};
}


std::vector<std::string> Planning::tokenize(const std::string &input) {
  std::vector<std::string> tokens;
  std::istringstream iss(input);
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

bool Planning::startPlanning() {
  if (!waitForService(clips_start_planning_client_, 5s)) {
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  std::mutex mtx;
  std::condition_variable cv;
  bool done = false;
  bool success = false;

  auto callback =
      [&](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        success = future.get()->success;
        done = true;
        cv.notify_one();
      };

  clips_start_planning_client_->async_send_request(request, callback);

  std::unique_lock<std::mutex> lock(mtx);
  if (!cv.wait_for(lock, 5s, [&] { return done; })) {
    RCLCPP_ERROR(node_->get_logger(), "Clips start planning timeout");
    return false;
  }

  return success;
}

std::string Planning::parseSemanticCommand(const std::string &command) {
  if (!waitForService(semantic_parser_client_, 5s)) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "semantic_parser service not available");
    return "";
  }

  auto request =
      std::make_shared<semantic_parser_interfaces::srv::Parser::Request>();
  request->command = command;

  std::mutex mtx;
  std::condition_variable cv;
  bool done = false;
  std::string result;

  auto callback =
      [&](rclcpp::Client<
          semantic_parser_interfaces::srv::Parser>::SharedFuture future) {
        auto response = future.get();
        result = response->response;   // usa el campo correcto de tu srv
        done = true;
        cv.notify_one();
      };

  semantic_parser_client_->async_send_request(request, callback);

  std::unique_lock<std::mutex> lock(mtx);
  if (!cv.wait_for(lock, 5s, [&] { return done; })) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Timeout calling semantic_parser service");
    return "";
  }

  return result;
}
