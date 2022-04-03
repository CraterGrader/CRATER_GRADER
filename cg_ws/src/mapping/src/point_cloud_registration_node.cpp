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
    point_cloud_map_(new pcl::PointCloud<pcl::PointXYZ>),
    initial_data_received_(false),
    new_data_received_(false) {
  // Initialize publishers and subscribers
  terrain_raw_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/raw_map", 1
  );
  // TODO change topic subscription back to /terrain/filtered
  filtered_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudRegistrationNode::filteredPointsCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PointCloudRegistrationNode::timerCallback, this)
  );
}

void PointCloudRegistrationNode::timerCallback() {
  if (!initial_data_received_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for initial point cloud data");
    return;
  }
  if (new_data_received_) {
    icp_.setInputSource(new_point_cloud_);
    icp_.setInputTarget(point_cloud_map_);
    pcl::PointCloud<pcl::PointXYZ> registered_cloud;
    icp_.align(registered_cloud);  // TODO insert initial transform guess here?
    if (icp_.hasConverged()) {
      RCLCPP_INFO(this->get_logger(), "ICP Converged");
      Eigen::Matrix4f registered_cloud_transform = icp_.getFinalTransformation();
      pcl::transformPointCloud(*new_point_cloud_, registered_cloud, registered_cloud_transform);
      *point_cloud_map_ += registered_cloud;

    } else {
      RCLCPP_WARN(this->get_logger(), "ICP Failed Convergence");
    }
    new_data_received_ = false;
  }
  sensor_msgs::msg::PointCloud2 point_cloud_map_msg;
  pcl::toROSMsg(*point_cloud_map_, point_cloud_map_msg);
  terrain_raw_map_pub_->publish(point_cloud_map_msg);
  RCLCPP_INFO(this->get_logger(), "Published map");
}

void PointCloudRegistrationNode::filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<T>
  pcl::fromROSMsg(*msg, *new_point_cloud_);
  if (!initial_data_received_) {
    pcl::fromROSMsg(*msg, *point_cloud_map_);
    initial_data_received_ = true;
  } else {
    new_data_received_ = true;
  }
}

}  // namespace mapping
}  // namespace cg
