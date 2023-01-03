#ifndef MINIMAL_ACTION_CLIENT__MINIMAL_ACTION_CLIENT_HPP_
#define MINIMAL_ACTION_CLIENT__MINIMAL_ACTION_CLIENT_HPP_

#include "minimal_action_client/visibility_control.h"
#include <cinttypes>
#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace minimal_action_client {
class MinimalActionClient : public rclcpp::Node {
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit MinimalActionClient(
      const std::string &action_name = "fibonacci",
      const std::string &node_name = "minimal_action_client",
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node(node_name, node_options) {

    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), action_name);

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           [this] { send_goal(); });
  }

  auto is_goal_done() const -> bool { return this->goal_done_; }

  void send_goal() {

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto &&PH1) {
      goal_response_callback_(std::forward<decltype(PH1)>(PH1));
    };
    send_goal_options.feedback_callback = [this](auto &&PH1, auto &&PH2) {
      feedback_callback_(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2));
    };
    send_goal_options.result_callback = [this](auto &&PH1) {
      result_callback_(std::forward<decltype(PH1)>(PH1));
    };
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_{};

  void goal_response_callback_(GoalHandleFibonacci::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback_(
      GoalHandleFibonacci::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(),
                "Next number in sequence received: %" PRId32,
                feedback->sequence.back());
  }

  void result_callback_(const GoalHandleFibonacci::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
    }
  }
}; // class MinimalActionClient

} // namespace minimal_action_client

#endif // MINIMAL_ACTION_CLIENT__MINIMAL_ACTION_CLIENT_HPP_
