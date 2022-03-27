#ifndef MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
#define MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cg {
namespace mapping {

class PointCloudRegistrationNode : public rclcpp::Node {

public:
  PointCloudRegistrationNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_raw_map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  void timerCallback();
  void filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};


}  // namespace mapping
}  // namespace cg

#endif  // MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
