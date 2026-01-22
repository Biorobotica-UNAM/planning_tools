#include "planning_tools/Planning.hpp"

using namespace std::chrono_literals;

Planning::Planning(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    interpret_client_ =
        node_->create_client<service::srv::String>(
            "listener/natural_to_clips/interpret");

    request_instruction_client_ =
        node_->create_client<std_srvs::srv::Trigger>(
            "/request_instruction");
}

bool Planning::waitForService(
    const rclcpp::ClientBase::SharedPtr& client,
    const std::string& service_name,
    std::chrono::seconds timeout)
{
    auto start = node_->now();

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Interrupted while waiting for service %s",
                service_name.c_str());
            return false;
        }

        if ((node_->now() - start).seconds() >= timeout.count()) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Timeout waiting for service %s",
                service_name.c_str());
            return false;
        }
    }

    return true;
}

std::string Planning::interpretToClips(const std::string& sentence)
{
    if (!waitForService(interpret_client_,
                        "listener/natural_to_clips/interpret",
                        5s)) {
        return "";
    }

    auto request = std::make_shared<service::srv::String::Request>();
    request->data = sentence;

    auto future = interpret_client_->async_send_request(request);

    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Timeout calling interpret service");
        return "";
    }

    auto response = future.get();
    return response->data;
}

bool Planning::requestInstruction()
{
    if (!waitForService(request_instruction_client_,
                        "/request_instruction",
                        5s)) {
        return false;
    }

    auto request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    auto future =
        request_instruction_client_->async_send_request(request);

    if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Timeout calling request_instruction");
        return false;
    }

    return future.get()->success;
}
