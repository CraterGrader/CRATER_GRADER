#include <rclcpp/rclcpp.hpp>
#include "planning/behavior_executive_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::planning::BehaviorExecutive>());
  rclcpp::shutdown();
  return 0;
}
