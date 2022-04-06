#include "mapping/point_cloud_registration_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

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
  filtered_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", rclcpp::SensorDataQoS(), std::bind(&PointCloudRegistrationNode::filteredPointsCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PointCloudRegistrationNode::timerCallback, this)
  );

  // Load parameters
  this->declare_parameter<std::string>("source_frame", "map");
  this->get_parameter("source_frame", source_frame_);
  this->declare_parameter<std::string>("target_frame", "base_link");
  this->get_parameter("target_frame", target_frame_);
  this->declare_parameter<int>("icp_max_iters", 50);
  this->get_parameter("icp_max_iters", icp_max_iters_);
  this->declare_parameter<float>("voxel_filter_size", 0.001);
  this->get_parameter("voxel_filter_size", voxel_filter_size_);

  icp_.setMaximumIterations(icp_max_iters_);
}

void PointCloudRegistrationNode::timerCallback() {
  if (!initial_data_received_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for initial point cloud data");
    return;
  }
  if (new_data_received_) {
    icp_.setInputSource(new_point_cloud_);
    icp_.setInputTarget(point_cloud_map_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp_.align(*registered_cloud);  // TODO insert initial transform guess here?
    if (icp_.hasConverged()) {
      RCLCPP_INFO(this->get_logger(), "ICP Converged");
      Eigen::Matrix4d registered_cloud_transform = icp_.getFinalTransformation().cast<double>();
      std::cout << registered_cloud_transform << std::endl;
      pcl::transformPointCloud(*new_point_cloud_, *registered_cloud, registered_cloud_transform);
      *point_cloud_map_ += *registered_cloud;

    } else {
      RCLCPP_WARN(this->get_logger(), "ICP Failed Convergence");
    }
    new_data_received_ = false;
  }
  RCLCPP_INFO(this->get_logger(), "Number of points before downsample: %lu", point_cloud_map_->size());
  // Downsample
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(point_cloud_map_);
  voxel_grid.setLeafSize(voxel_filter_size_, voxel_filter_size_, voxel_filter_size_);
  voxel_grid.filter(*point_cloud_map_);
  RCLCPP_INFO(this->get_logger(), "Number of points after downsample: %lu", point_cloud_map_->size());

  // Convert map to sensor_msgs::msg::PointCloud2 and publish message
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
