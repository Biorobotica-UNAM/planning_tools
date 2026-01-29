#include "planning_tools/Planning.hpp"

using namespace std::chrono_literals;

Planning::Planning(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    interpret_client_ =
        node_->create_client<clips_interfaces::srv::String>(
            "/hri/listener/natural_to_clips/interpret");

    request_instruction_client_ =
        node_->create_client<std_srvs::srv::Trigger>(
            "/request_instruction");

    clips_start_planning_ =
        node_->create_client<std_srvs::srv::Trigger>(
            "/clips/start_planning");
}

bool Planning::waitForService(
    const rclcpp::ClientBase::SharedPtr& client,
    const std::chrono::seconds& timeout) const
{
    return client->wait_for_service(timeout);
}

std::string Planning::interpretToClips(const std::string& sentence)
{
    if (!waitForService(interpret_client_, 5s)) {
        return "";
    }

    auto request = std::make_shared<clips_interfaces::srv::String::Request>();
    request->data = sentence;

    auto future = interpret_client_->async_send_request(request);

    auto state = rclcpp::spin_until_future_complete(
        node_, future, 5s);

    if (state != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Timeout calling interpret service");
        return "";
    }

    auto response = future.get();
    return response->data;
}

std::vector<std::string> Planning::requestInstruction()
{
    if (!waitForService(request_instruction_client_, 5s)) {
        return {};
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (rclcpp::ok()) 
    {
        auto future = request_instruction_client_->async_send_request(request);

        auto state = rclcpp::spin_until_future_complete(node_, future, 5s);

        if (state != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(),
                        "Waiting for instruction...");
            rclcpp::sleep_for(300ms);
            continue;
        }

        auto response = future.get();

        if (response->success && !response->message.empty()) {
            RCLCPP_INFO(node_->get_logger(),
                        "Instruction received: %s",
                        response->message.c_str());
            return tokenize(response->message);
        }

        rclcpp::sleep_for(300ms);
    }

    return {};
}


std::vector<std::string> Planning::tokenize(const std::string& input)
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
    if (!waitForService(clips_start_planning_, 5s)) {
        return false;
    }

    auto request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    auto future =
        clips_start_planning_->async_send_request(request);

    auto state = rclcpp::spin_until_future_complete(node_, future, 5s);

    if (state != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Clips instruction timeout");
        return false;
    }

    auto response = future.get();
    return response->success;
}
