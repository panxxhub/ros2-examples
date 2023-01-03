#include "minimal_action_client/minimal_action_client.hpp"
#include <rclcpp/rclcpp.hpp>

auto main(int argc, char **argv) -> int {
  rclcpp::init(argc, argv);
  auto action_client =
      std::make_shared<minimal_action_client::MinimalActionClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}