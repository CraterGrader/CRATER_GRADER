#include "mapping/point_cloud_registration_node_but_fancier.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace cg {
namespace mapping {

PointCloudRegistrationNodeFancy::PointCloudRegistrationNodeFancy() :
    Node("point_cloud_registration_node"),
    new_point_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
    point_cloud_map_(new pcl::PointCloud<pcl::PointXYZ>),
    new_data_received_(false) {
  // Initialize publishers and subscribers
  terrain_raw_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/raw_map", 1
  );
  filtered_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&PointCloudRegistrationNodeFancy::filteredPointsCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PointCloudRegistrationNodeFancy::timerCallback, this)
  );
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load parameters
  this->declare_parameter<std::string>("source_frame", "realsense_frame");
  this->get_parameter("source_frame", source_frame_);
  this->declare_parameter<std::string>("target_frame", "map");
  this->get_parameter("target_frame", target_frame_);
  this->declare_parameter<int>("icp_max_iters", 50);
  this->get_parameter("icp_max_iters", icp_max_iters_);
  this->declare_parameter<float>("voxel_filter_size", 0.001);
  this->get_parameter("voxel_filter_size", voxel_filter_size_);


  // Statistical Outlier Removal Params
  this->declare_parameter<int>("sor_mean_k", 50);
  this->get_parameter("sor_mean_k", sor_mean_k_);
  this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
  this->get_parameter("sor_stddev_mul_thresh", sor_stddev_mul_thresh_);


  icp_.setMaximumIterations(icp_max_iters_);
  // icp_.setMaxCorrespondenceDistance (0.05);

  RCLCPP_INFO(this->get_logger(), "Using parameters: ");
  RCLCPP_INFO(this->get_logger(), "* source_frame: %s", source_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "* target_frame: %s", target_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "* icp_max_iters: %d", icp_max_iters_);
  RCLCPP_INFO(this->get_logger(), "* voxel_filter_size: %f", voxel_filter_size_);
}

void PointCloudRegistrationNodeFancy::timerCallback() {
  if (point_cloud_map_->size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for initial point cloud data");
    return;
  }

  if (new_data_received_) {

    // Statistical outlier removal for salt-and-pepper noise
    // https://pcl.readthedocs.io/en/latest/statistical_outlier.html
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(new_point_cloud_);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_mul_thresh_);
    sor.filter(*new_point_cloud_);

    pcl::PointCloud<pcl::PointXYZ>& source_cloud = *new_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ>& target_cloud = *point_cloud_map_;

    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud(*new_point_cloud_, *new_point_cloud_, nan_idx);

    // Estimate cloud normals

    // Calculate normals from new point cloud (source)
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr new_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& new_normals = *new_normals_ptr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(new_point_cloud_);
    ne.setSearchMethod(tree_xyz);
    ne.setRadiusSearch(0.05); // TODO: Need to tune this param
    ne.compute(*new_normals_ptr);
    for(size_t i = 0;  i < new_normals.points.size(); ++i) {
        new_normals.points[i].x = source_cloud.points[i].x;
        new_normals.points[i].y = source_cloud.points[i].y;
        new_normals.points[i].z = source_cloud.points[i].z;
    }

    // Calculate normals from target point cloud (existing map)
    pcl::PointCloud<pcl::PointNormal>::Ptr map_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& map_normals = *map_normals_ptr;
    ne.setInputCloud(point_cloud_map_);
    ne.compute(*map_normals_ptr);
    for(size_t i = 0;  i < map_normals.points.size(); ++i) {
        map_normals.points[i].x = target_cloud.points[i].x;
        map_normals.points[i].y = target_cloud.points[i].y;
        map_normals.points[i].z = target_cloud.points[i].z;
    }


    //SIFT Keypoint Estimation

    // RUN ICP
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

  //   // Estimate the SIFT keypoints
  //   pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  //   pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
  //   pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
  //   pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
  //   sift.setSearchMethod(tree_normal);
  //   sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  //   sift.setMinimumContrast(min_contrast);
  //   sift.setInputCloud(src_normals_ptr);
  //   sift.compute(src_keypoints);
  //   std::cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";
  
  //   pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
  //   pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
  //   sift.setInputCloud(tar_normals_ptr);
  //   sift.compute(tar_keypoints);
  //   std::cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";
    
  //   // Extract FPFH features from SIFT keypoints
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  //   pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
  //   // pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  //   // fpfh.setSearchSurface (point_cloud_map_);
  //   // fpfh.setInputCloud (src_keypoints_xyz);
  //   // fpfh.setInputNormals (src_normals_ptr);
  //   // fpfh.setSearchMethod (tree_xyz);
  //   // pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  //   // pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
  //   // fpfh.setRadiusSearch(0.05);
  //   // fpfh.compute(src_features);
  //   // std::cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

  //   pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  //   pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
  //   // fpfh.setSearchSurface (new_point_cloud_);
  //   // fpfh.setInputCloud (tar_keypoints_xyz);
  //   // fpfh.setInputNormals (tar_normals_ptr);
  //   // pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  //   // pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
  //   // fpfh.compute(tar_features);
  //   // std::cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
    
  //   // Compute the transformation matrix for alignment
  //   Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
  //   // tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
  //   //         tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
  //   // Uncomment this code to run ICP 
  //   // tform = refineAlignment (point_cloud_map_, new_point_cloud_, tform, max_correspondence_distance,
  //   //         outlier_rejection_threshold, transformation_epsilon, max_iterations);

  //   tform = refineAlignment (new_point_cloud_, point_cloud_map_, tform, max_correspondence_distance,
  //           outlier_rejection_threshold, transformation_epsilon, max_iterations);

  //   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<pcl::PointXYZ>& transformed_cloud = *transformed_cloud_ptr;
  //   //pcl::transformPointCloud(*new_point_cloud_, transformed_cloud, tform);

  //   *point_cloud_map_ += *new_point_cloud_;
  //   // End of New Stuff

  //   new_data_received_ = false;
  // }

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

void PointCloudRegistrationNodeFancy::filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

    // Filter outliers from first point cloud
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(new_point_cloud_);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_mul_thresh_);
    sor.filter(*new_point_cloud_);

    pcl::transformPointCloud(*new_point_cloud_, *point_cloud_map_, tf2::transformToEigen(tf_).matrix().cast<float>());
    // first_pc = true;
  } else {
    new_data_received_ = true;
  }
}

}  // namespace mapping
}  // namespace cg
