#include "mapping/point_cloud_registration_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/statistical_outlier_removal.h>


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
  this->declare_parameter<std::string>("source_frame", "realsense_frame");
  this->get_parameter("source_frame", source_frame_);
  this->declare_parameter<std::string>("target_frame", "map");
  this->get_parameter("target_frame", target_frame_);

  // ICP Params
  this->declare_parameter<int>("icp_max_iters", 10);
  this->get_parameter("icp_max_iters", icp_max_iters_);
  this->declare_parameter<double>("icp_max_correspondence_dist", std::sqrt(std::numeric_limits<double>::max()));
  this->get_parameter("icp_max_correspondence_dist", icp_max_correspondence_dist_);
  this->declare_parameter<double>("icp_transformation_eps", 0.0);
  this->get_parameter("icp_transformation_eps", icp_transformation_eps_);
  this->declare_parameter<double>("icp_euclidean_fitness_eps", -std::numeric_limits<double>::max());
  this->get_parameter("icp_euclidean_fitness_eps", icp_euclidean_fitness_epsilon_);
  // this->declare_parameter<int>("icp_min_num_correspondences", 3);
  // this->get_parameter("icp_min_num_correspondences", icp_min_num_correspondences_);

  // Statistical Outlier Removal Params
  this->declare_parameter<int>("sor_mean_k", 50);
  this->get_parameter("sor_mean_k", sor_mean_k_);
  this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
  this->get_parameter("sor_stddev_mul_thresh", sor_stddev_mul_thresh_);

  // Downsampling Params
  this->declare_parameter<float>("voxel_filter_size", 0.01);
  this->get_parameter("voxel_filter_size", voxel_filter_size_);

  icp_.setMaximumIterations(icp_max_iters_);
  icp_.setMaxCorrespondenceDistance(icp_max_correspondence_dist_);
  icp_.setTransformationEpsilon(icp_transformation_eps_);
  icp_.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
  // icp_.setMin

  RCLCPP_INFO(this->get_logger(), "Using parameters: ");
  RCLCPP_INFO(this->get_logger(), "* source_frame: %s", source_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "* target_frame: %s", target_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "* icp_max_iters: %d", icp_.getMaximumIterations());
  RCLCPP_INFO(this->get_logger(), "* icp_max_correspondence_dist: %f", icp_.getMaxCorrespondenceDistance());
  RCLCPP_INFO(this->get_logger(), "* icp_transformation_eps: %f", icp_.getTransformationEpsilon());
  RCLCPP_INFO(this->get_logger(), "* icp_euclidean_fitness_eps: %f", icp_.getEuclideanFitnessEpsilon());
  RCLCPP_INFO(this->get_logger(), "* sor_mean_k: %d", sor_mean_k_);
  RCLCPP_INFO(this->get_logger(), "* sor_stddev_mul_thresh: %f", sor_stddev_mul_thresh_);
  RCLCPP_INFO(this->get_logger(), "* voxel_filter_size: %f", voxel_filter_size_);
}

void PointCloudRegistrationNode::timerCallback() {
  if (point_cloud_map_->size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for initial point cloud data");
    return;
  }

  if (new_data_received_) {
    // Plane removal test
    // https://pointclouds.org/documentation/tutorials/planar_segmentation.html
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);

    seg.setInputCloud (new_point_cloud_);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) {
      RCLCPP_WARN (this->get_logger(), "Could not estimate a planar model for the given dataset.");
    } else {
      // https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(new_point_cloud_);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*new_point_cloud_);
    }

    // Statistical outlier removal for salt-and-pepper noise
    // https://pcl.readthedocs.io/en/latest/statistical_outlier.html
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(new_point_cloud_);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_mul_thresh_);
    sor.filter(*new_point_cloud_);

    icp_.setInputSource(new_point_cloud_);
    icp_.setInputTarget(point_cloud_map_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    matrix_ = tf2::transformToEigen(tf_);
    std::cout << matrix_.matrix().cast<float>() << std::endl;
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
  point_cloud_map_msg.header.frame_id = target_frame_; 
  terrain_raw_map_pub_->publish(point_cloud_map_msg);
  RCLCPP_INFO(this->get_logger(), "Published map");
}

void PointCloudRegistrationNode::filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<T>
  pcl::fromROSMsg(*msg, *new_point_cloud_);
  try {
    tf_ = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
    // std::cout << tf2::transformToEigen(transform_stamped).matrix().cast<float>() << std::endl;
    // pcl::transformPointCloud(*new_point_cloud_, *point_cloud_map_, tf2::transformToEigen(transform_stamped).matrix().cast<float>());
    // pcl::fromROSMsg(*msg, *point_cloud_map_);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not lookupTransform from source %s to target %s: %s",
      source_frame_.c_str(), target_frame_.c_str(), ex.what()
    );
    return;
  }
 
  if (point_cloud_map_->size() == 0) {
    pcl::transformPointCloud(*new_point_cloud_, *point_cloud_map_, tf2::transformToEigen(tf_).matrix().cast<float>());
  } else {
    new_data_received_ = true;
  }
}

//BEN COMMENT 

}  // namespace mapping
}  // namespace cg
