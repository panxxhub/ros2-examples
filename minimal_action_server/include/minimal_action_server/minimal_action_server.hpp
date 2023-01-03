#ifndef MINIMAL_ACTION_SERVER__MINIMAL_ACTION_SERVER_HPP_
#define MINIMAL_ACTION_SERVER__MINIMAL_ACTION_SERVER_HPP_

#include "minimal_action_server/visibility_control.h"
#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace minimal_action_server {
class MinimalActionServer : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit MinimalActionServer(
      const std::string &name = "minimal_action_server",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node(name, options) {

    // NOLINTBEGIN

    using namespace std::placeholders;
    auto action_name = std::string(this->get_name()) + "/fibonacci";

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), action_name,

        std::bind(&MinimalActionServer::handle_goal_, this, _1, _2),
        std::bind(&MinimalActionServer::handle_cancel_, this, _1),
        std::bind(&MinimalActionServer::handle_accepted_, this, _1));

    // NOLINTEND
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  auto handle_goal_(const rclcpp_action::GoalUUID &uuid,
                    std::shared_ptr<const Fibonacci::Goal> goal)
      -> rclcpp_action::GoalResponse {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d",
                goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  auto handle_cancel_(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
      -> rclcpp_action::CancelResponse {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute_(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void
  handle_accepted_(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up
    // a new thread
    std::thread{
        [this](auto &&PH1) { execute_(std::forward<decltype(PH1)>(PH1)); },
        goal_handle}
        .detach();
  }
}; // class MinimalActionServer

} // namespace minimal_action_server

#endif // MINIMAL_ACTION_SERVER__MINIMAL_ACTION_SERVER_HPP_
