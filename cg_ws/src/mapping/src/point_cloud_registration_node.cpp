#include "mapping/point_cloud_registration_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cg {
namespace mapping {

PointCloudRegistrationNode::PointCloudRegistrationNode() :
    Node("point_cloud_registration_node"),
    new_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
    point_cloud_map_(new pcl::PointCloud<pcl::PointXYZ>) {
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
  icp_.setInputSource(new_point_cloud_);
  icp_.setInputTarget(point_cloud_map_);
  pcl::PointCloud<pcl::PointXYZ> fused_cloud;
  icp_.align(fused_cloud);
  if (icp_.hasConverged()) {
    point_cloud_map_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(fused_cloud);
    sensor_msgs::msg::PointCloud2 point_cloud_map_msg;
    pcl::toROSMsg(*point_cloud_map_, point_cloud_map_msg);
    terrain_raw_map_pub_->publish(point_cloud_map_msg);
  } else {
    RCLCPP_WARN(this->get_logger(), "ICP Failed Convergence");
  }
}

void PointCloudRegistrationNode::filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<T>
  pcl::fromROSMsg(*msg, *new_point_cloud_);
}

}  // namespace mapping
}  // namespace cg
