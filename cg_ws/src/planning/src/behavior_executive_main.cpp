#include <rclcpp/rclcpp.hpp>
#include "planning/behavior_executive_node.hpp"

int main(int argc, char *argv[])
{

  // Use multithreaded executor for service call in timer callback: https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<cg::planning::BehaviorExecutive>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);

  RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;
}
