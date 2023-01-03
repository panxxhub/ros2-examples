#include "minimal_action_client/minimal_action_client.hpp"
#include <rclcpp/rclcpp.hpp>

auto main(int argc, char **argv) -> int {
  rclcpp::init(argc, argv);
  auto client_0x1 =
      std::make_shared<minimal_action_client::MinimalActionClient>(
          "/fib_0x1/fibonacci", "client_0x1");
  auto client_0x2 =
      std::make_shared<minimal_action_client::MinimalActionClient>(
          "/fib_0x2/fibonacci", "client_0x2");

  while (!(client_0x1->is_goal_done() && client_0x2->is_goal_done())) {
    rclcpp::spin_some(client_0x1);
    rclcpp::spin_some(client_0x2);
  }

  rclcpp::shutdown();
  return 0;
}