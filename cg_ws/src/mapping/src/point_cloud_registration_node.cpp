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
    new_data_received_(false) {
  // Initialize publishers and subscribers
  terrain_raw_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/raw_map", 1
  );
  filtered_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudRegistrationNode::filteredPointsCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PointCloudRegistrationNode::timerCallback, this)
  );
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load parameters
  this->declare_parameter<std::string>("source_frame", "map");
  this->get_parameter("source_frame", source_frame_);
  this->declare_parameter<std::string>("target_frame", "realsense_frame");
  this->get_parameter("target_frame", target_frame_);
  this->declare_parameter<int>("icp_max_iters", 50);
  this->get_parameter("icp_max_iters", icp_max_iters_);
  this->declare_parameter<float>("voxel_filter_size", 0.001);
  this->get_parameter("voxel_filter_size", voxel_filter_size_);

  icp_.setMaximumIterations(icp_max_iters_);

  RCLCPP_INFO(this->get_logger(), "Using parameters: ");
  RCLCPP_INFO(this->get_logger(), "* icp_max_iters: %d", icp_max_iters_);
  RCLCPP_INFO(this->get_logger(), "* voxel_filter_size: %f", voxel_filter_size_);
}

void PointCloudRegistrationNode::timerCallback() {
  if (point_cloud_map_->size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for initial point cloud data");
    return;
  }
  // sensor_msgs::msg::PointCloud2 point_cloud_map_msg_temp;
  // pcl::toROSMsg(*point_cloud_map_, point_cloud_map_msg_temp);
  // point_cloud_map_msg_temp.header.frame_id = "odom";
  // terrain_raw_map_pub_->publish(point_cloud_map_msg_temp);
  // return;

  if (new_data_received_) {

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("odom", "realsense_frame", tf2::TimePointZero);
    }

    catch (tf2::TransformException &ex){
      tf.transform.translation.x = 0;
      tf.transform.translation.y = 0;
      tf.transform.translation.z = 0;
      tf.transform.rotation.x = 0;
      tf.transform.rotation.y = 0;
      tf.transform.rotation.z = 0;
      tf.transform.rotation.w = 1;
    }

    icp_.setInputSource(new_point_cloud_);
    icp_.setInputTarget(point_cloud_map_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    matrix_ = tf2::transformToEigen(tf); 
    icp_.align(*registered_cloud, matrix_.matrix().cast<float>());  
    if (icp_.hasConverged()) {
      // RCLCPP_INFO(this->get_logger(), "ICP Converged");
      Eigen::Matrix4d registered_cloud_transform = icp_.getFinalTransformation().cast<double>();
      // std::cout << registered_cloud_transform << std::endl;
      pcl::transformPointCloud(*new_point_cloud_, *registered_cloud, registered_cloud_transform);
      *point_cloud_map_ += *registered_cloud;

    } else {
      RCLCPP_WARN(this->get_logger(), "ICP Failed Convergence");
    }
    new_data_received_ = false;
  }

  // Downsample
  RCLCPP_INFO(this->get_logger(), "Number of points before downsample: %lu", point_cloud_map_->size());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(point_cloud_map_);
  voxel_grid.setLeafSize(voxel_filter_size_, voxel_filter_size_, voxel_filter_size_);
  voxel_grid.filter(*point_cloud_map_);
  RCLCPP_INFO(this->get_logger(), "Number of points after downsample: %lu", point_cloud_map_->size());

  // Convert map to sensor_msgs::msg::PointCloud2 and publish message
  sensor_msgs::msg::PointCloud2 point_cloud_map_msg;
  pcl::toROSMsg(*point_cloud_map_, point_cloud_map_msg);
  point_cloud_map_msg.header.frame_id = "odom"; 
  terrain_raw_map_pub_->publish(point_cloud_map_msg);
  RCLCPP_INFO(this->get_logger(), "Published map");
}

void PointCloudRegistrationNode::filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<T>
  pcl::fromROSMsg(*msg, *new_point_cloud_);
  if (point_cloud_map_->size() == 0) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("odom", "realsense_frame", tf2::TimePointZero);
      std::cout << tf2::transformToEigen(transform_stamped).matrix().cast<float>() << std::endl;
      pcl::transformPointCloud(*new_point_cloud_, *point_cloud_map_, tf2::transformToEigen(transform_stamped).matrix().cast<float>());
      // pcl::fromROSMsg(*msg, *point_cloud_map_);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not lookupTransform from source %s to target %s: %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what()
      );
    }

  } else {
    new_data_received_ = true;
  }
}

}  // namespace mapping
}  // namespace cg
