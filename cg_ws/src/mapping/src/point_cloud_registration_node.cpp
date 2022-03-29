#include "mapping/point_cloud_registration_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace cg {
namespace mapping {

PointCloudRegistrationNode::PointCloudRegistrationNode() : Node("point_cloud_registration_node") {
  // Initialize publishers and subscribers
  terrain_raw_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/raw_map", 1
  );
  filtered_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", rclcpp::SensorDataQoS(), std::bind(&PointCloudRegistrationNode::filteredPointsCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PointCloudRegistrationNode::timerCallback, this)
  );
}

void PointCloudRegistrationNode::timerCallback() {
  // TODO
}

void PointCloudRegistrationNode::filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // TODO
}

}  // namespace mapping
}  // namespace cg
