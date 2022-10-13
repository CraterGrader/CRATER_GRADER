#include <rclcpp/rclcpp.hpp>
#include "motion_control/motion_control_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::motion_control::MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}