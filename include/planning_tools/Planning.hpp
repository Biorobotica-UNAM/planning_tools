#pragma once

/**
 * @brief Facade over ROS2 planning pipeline services.
 *
 * Responsibility: provide a clean, synchronous interface to planning
 * services (semantic parsing, CLIPS interpretation, instruction
 * retrieval) so callers never need to know about ROS2 async patterns,
 * callback groups, or timeouts.
 *
 * Design patterns applied:
 *   Facade          (GoF Structural)  — hides ROS2 service clients
 *                                       behind intention-revealing methods.
 *   Template Method (GoF Behavioral)  — call_service<T>() captures the
 *                                       invariant async→wait→return flow;
 *                                       each public method supplies only
 *                                       what varies (request + response
 * mapping).
 *
 * Principles applied:
 *   SRP  — one class, one responsibility: planning communication.
 *          File loading is extracted to a private helper.
 *   DRY  — the async + future.wait_for + timeout pattern lives in
 *          exactly one place.
 *   DIP  — callers depend on Planning, not on rclcpp::Client directly.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "clips_interfaces/srv/string.hpp"
#include "semantic_parser_interfaces/srv/extract_entity.hpp"
#include "semantic_parser_interfaces/srv/parser.hpp"
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class Planning {
public:
  explicit Planning(const rclcpp::Node::SharedPtr &node);

  /**
   * Translate a natural-language sentence into a CLIPS fact string.
   * @param sentence  Raw natural-language input.
   * @param timeout   Maximum time to wait for the service (default: 5 s).
   * @return CLIPS fact string, or empty string on failure.
   */
  std::string interpretToClips(const std::string &sentence,
                               std::chrono::seconds timeout = 5s);

  /**
   * Convert a primitive action sentence to a CLIPS representation.
   * @param sentence  Primitive action string.
   * @param timeout   Maximum time to wait for the service (default: 5 s).
   * @return CLIPS representation string, or empty string on failure.
   */
  std::string primitiveToClips(const std::string &sentence,
                               std::chrono::seconds timeout = 5s);

  /**
   * Extract a sustantive from a desired category.
   * @param sentence  Natural language sentence.
   * @param category  The category of the object which is meant to be saved.
   * @param timeout   Maximum time to wait for the service (default: 5 s).
   * @return std::string Only the name of the sustantive of the desired
   * category.
   */
  std::vector<std::string> extractEntity(const std::string &sentence,
                                         const std::string &category,
                                         std::chrono::seconds timeout = 5s);

  /**
   * Poll the instruction service until a valid instruction is received.
   * Retries every 300 ms while rclcpp::ok().
   * @param timeout   Per-call timeout (default: 5 s).
   * @return Tokenised instruction vector, or empty vector on shutdown.
   */
  std::vector<std::string>
  requestInstruction(std::chrono::seconds timeout = 5s);

  /**
   * Trigger the CLIPS planning engine.
   * @param timeout   Maximum time to wait for the service (default: 5 s).
   * @return true if planning started successfully.
   */
  bool startPlanning(std::chrono::seconds timeout = 5s);

  /**
   * Run the semantic parser on a command string or a .txt file path.
   * If @p input is an existing .txt file, its contents are loaded first.
   * @param input    Raw command string or path to a .txt file.
   * @param timeout  Maximum time to wait for the service (default: 10 s).
   * @return Semantic CD string, or empty string on failure.
   */
  std::string parseSemanticCommand(const std::string &input,
                                   std::chrono::seconds timeout = 10s);

private:
  // ── ROS2 infrastructure ───────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

  rclcpp::Client<clips_interfaces::srv::String>::SharedPtr interpret_client_;
  rclcpp::Client<clips_interfaces::srv::String>::SharedPtr primitive_client_;
  rclcpp::Client<semantic_parser_interfaces::srv::ExtractEntity>::SharedPtr
      entity_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_instruction_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      clips_start_planning_client_;
  rclcpp::Client<semantic_parser_interfaces::srv::Parser>::SharedPtr
      semantic_parser_client_;

  // ── Template Method: invariant call flow ──────────────────────────────────

  /**
   * Template Method: invariant async → wait → return flow.
   *
   * Encapsulates the wait_for_service + async_send_request +
   * future.wait_for + timeout pattern that would otherwise be
   * duplicated in every service call.
   *
   * Uses future.wait_for() (not mutex+cv) because all clients share a
   * Reentrant CallbackGroup, which allows blocking waits inside callbacks.
   *
   * @tparam ServiceT   ROS2 service type.
   * @tparam CallbackT  Callable: (Response::SharedPtr) → void.
   *                    Reads the response and stores results into
   *                    captured variables.
   *
   * @param client      The service client to use.
   * @param request     The fully populated request object.
   * @param on_response Callback that extracts results from the response.
   * @param timeout     Maximum time allowed for the call.
   * @param label       Human-readable service name for log messages.
   *
   * @return true if the call completed before the timeout.
   */
  template <typename ServiceT, typename CallbackT>
  bool call_service(typename rclcpp::Client<ServiceT>::SharedPtr &client,
                    typename ServiceT::Request::SharedPtr request,
                    CallbackT on_response, std::chrono::seconds timeout,
                    const std::string &label);

  // ── Private helpers ───────────────────────────────────────────────────────

  /**
   * Split a whitespace-delimited string into tokens.
   * Extracted to keep requestInstruction() focused on service communication.
   */
  std::vector<std::string> tokenize(const std::string &input) const;

  /**
   * Load the text content of a file into a string.
   * Extracted from parseSemanticCommand() to respect SRP:
   * file I/O and service communication are separate responsibilities.
   *
   * @param path  Absolute or relative path to a .txt file.
   * @return File contents, or empty string on failure.
   */
  std::string load_text_file(const std::string &path) const;

  /**
   * Return true if @p input is a path to an existing .txt file.
   * Extracted to give the condition an intention-revealing name.
   */
  bool is_txt_file(const std::string &input) const;
};

// ============================================================
// Template implementation (must live in the header)
// ============================================================

template <typename ServiceT, typename CallbackT>
bool Planning::call_service(
    typename rclcpp::Client<ServiceT>::SharedPtr &client,
    typename ServiceT::Request::SharedPtr request, CallbackT on_response,
    std::chrono::seconds timeout, const std::string &label) {
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "%s service not available.",
                 label.c_str());
    return false;
  }

  auto future = client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling %s service after %ld s.",
                 label.c_str(), timeout.count());
    return false;
  }

  on_response(future.get());
  return true;
}
