#include "mapping/terrain_filtering_node.hpp"


namespace cg {
namespace mapping {

TerrainFilteringNode::TerrainFilteringNode() : Node("terrain_filtering_node") {
  // Initialize publishers and subscribers

  RCLCPP_INFO(this->get_logger(), "constructor");

  filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1
  );
  raw_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&TerrainFilteringNode::rawPointsCallback, this, std::placeholders::_1)
  );
}

void TerrainFilteringNode::rawPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // TODO

  RCLCPP_INFO(this->get_logger(), "Publishing");

  // go through all points and remove points 



}

}  // namespace mapping
}  // namespace cg
