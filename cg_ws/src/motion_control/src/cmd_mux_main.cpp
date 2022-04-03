#include <rclcpp/rclcpp.hpp>
#include "motion_control/cmdmux_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::cmdmux::CmdMuxNode>());
  rclcpp::shutdown();
  return 0;
}