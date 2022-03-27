#ifndef MAPPING__TERRAIN_FILTER_NODE_HPP
#define MAPPING__TERRAIN_FILTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cg {
namespace mapping {

class TerrainFilteringNode : public rclcpp::Node {

public:
  TerrainFilteringNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_points_sub_;

  /* Callbacks */
  void rawPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};


}  // namespace mapping
}  // namespace cg

#endif  // MAPPING__TERRAIN_FILTER_NODE_HPP
