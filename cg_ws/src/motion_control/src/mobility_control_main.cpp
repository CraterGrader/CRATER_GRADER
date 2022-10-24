#include <rclcpp/rclcpp.hpp>
#include "motion_control/mobility_control_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::motion_control::MobilityControlNode>());
  rclcpp::shutdown();
  return 0;
}