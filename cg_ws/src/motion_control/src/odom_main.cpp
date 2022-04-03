#include <rclcpp/rclcpp.hpp>
#include "motion_control/odom_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::odom::OdomNode>());
  rclcpp::shutdown();
  return 0;
}