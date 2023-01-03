#include "minimal_action_server/minimal_action_server.hpp"
#include <rclcpp/rclcpp.hpp>

auto main(int argc, char **argv) -> int {
  rclcpp::init(argc, argv);

  auto fib_0x1 =
      std::make_shared<minimal_action_server::MinimalActionServer>("fib_0x1");
  auto fib_0x2 =
      std::make_shared<minimal_action_server::MinimalActionServer>("fib_0x2");
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(fib_0x1);
  executor->add_node(fib_0x2);

  executor->spin();

  //   rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}