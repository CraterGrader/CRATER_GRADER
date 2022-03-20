#include "mapping/point_cloud_registration_node.hpp"

namespace cg {
namespace mapping {

PointCloudRegistrationNode::PointCloudRegistrationNode() : Node("point_cloud_registration_node") {
  // Initialize publishers and subscribers
  terrain_raw_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/raw_map", 1
  );
  raw_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", 1, std::bind(&PointCloudRegistrationNode::rawPointsCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PointCloudRegistrationNode::timerCallback, this)
  );
}

void PointCloudRegistrationNode::timerCallback() {
  // TODO
}

void PointCloudRegistrationNode::rawPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // TODO
}

}  // namespace mapping
}  // namespace cg
