#include <rclcpp/rclcpp.hpp>
#include "cg_visualization/marker_viz_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::cg_visualization::MarkerVizNode>());
  rclcpp::shutdown();
  return 0;
}