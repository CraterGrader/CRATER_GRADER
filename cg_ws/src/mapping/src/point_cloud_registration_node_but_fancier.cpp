#include "mapping/point_cloud_registration_node_but_fancier.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

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
  // sensor_msgs::msg::PointCloud2 point_cloud_map_msg_temp;
  // pcl::toROSMsg(*point_cloud_map_, point_cloud_map_msg_temp);
  // point_cloud_map_msg_temp.header.frame_id = "map";
  // terrain_raw_map_pub_->publish(point_cloud_map_msg_temp);
  // return;

  if (new_data_received_) {

    // geometry_msgs::msg::TransformStamped tf;
    // try {
    //   tf = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
    // }

    // catch (tf2::TransformException &ex){
    //   tf.transform.translation.x = 0;
    //   tf.transform.translation.y = 0;
    //   tf.transform.translation.z = 0;
    //   tf.transform.rotation.x = 0;
    //   tf.transform.rotation.y = 0;
    //   tf.transform.rotation.z = 0;
    //   tf.transform.rotation.w = 1;
    // }

    // New Stuff

    pcl::PointCloud<pcl::PointXYZ>& source_cloud = *new_point_cloud_;
    pcl::PointCloud<pcl::PointXYZ>& target_cloud = *point_cloud_map_;

    // Remove NaN points from point clouds
    // (this is necessary to avoid a segfault when running ICP)
    // std::vector<int> nan_idx;
    // pcl::removeNaNFromPointCloud(source_cloud, source_cloud, nan_idx);
    // pcl::removeNaNFromPointCloud(target_cloud, target_cloud, nan_idx);

    // Estimate cloud normals
    std::cout << "Computing source cloud normals\n";
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(new_point_cloud_);
    ne.setSearchMethod(tree_xyz);
    ne.setRadiusSearch(0.05); // TODO: Need to tune this param
    ne.compute(*src_normals_ptr);
    for(size_t i = 0;  i < src_normals.points.size(); ++i) {
        src_normals.points[i].x = source_cloud.points[i].x;
        src_normals.points[i].y = source_cloud.points[i].y;
        src_normals.points[i].z = source_cloud.points[i].z;
    }

    std::cout << "Computing target cloud normals\n";
    pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
    ne.setInputCloud(point_cloud_map_);
    ne.compute(*tar_normals_ptr);
    for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
        tar_normals.points[i].x = target_cloud.points[i].x;
        tar_normals.points[i].y = target_cloud.points[i].y;
        tar_normals.points[i].z = target_cloud.points[i].z;
    }

    // Estimate the SIFT keypoints
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree_normal);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(src_normals_ptr);
    sift.compute(src_keypoints);
    std::cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";
  
    pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
    sift.setInputCloud(tar_normals_ptr);
    sift.compute(tar_keypoints);
    std::cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";
    
    // Extract FPFH features from SIFT keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
    pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchSurface (point_cloud_map_);
    fpfh.setInputCloud (src_keypoints_xyz);
    fpfh.setInputNormals (src_normals_ptr);
    fpfh.setSearchMethod (tree_xyz);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(src_features);
    std::cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
    pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
    fpfh.setSearchSurface (new_point_cloud_);
    fpfh.setInputCloud (tar_keypoints_xyz);
    fpfh.setInputNormals (tar_normals_ptr);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
    fpfh.compute(tar_features);
    std::cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
    
    // Compute the transformation matrix for alignment
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
            tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
    
    // Uncomment this code to run ICP 
    // tform = refineAlignment (point_cloud_map_, new_point_cloud_, tform, max_correspondence_distance,
    //         outlier_rejection_threshold, transformation_epsilon, max_iterations);
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& transformed_cloud = *transformed_cloud_ptr;
    pcl::transformPointCloud(*point_cloud_map_, transformed_cloud, tform);
    std::cout << "Calculated transformation\n";

    *point_cloud_map_ += transformed_cloud;
    // if (first_pc) 
    // {
    //   *point_cloud_map_ = *transformed_cloud_ptr;
    //   first_pc = false;
    // } else 
    // {
    //   *point_cloud_map_ += *transformed_cloud_ptr;
    // }
    // End of New Stuff


    // // Old Stuff::
    // icp_.setInputSource(new_point_cloud_);
    // icp_.setInputTarget(point_cloud_map_);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // matrix_ = tf2::transformToEigen(tf_);
    // std::cout << matrix_.matrix().cast<float>() << std::endl;
    // icp_.align(*registered_cloud, matrix_.matrix().cast<float>());  
    // if (icp_.hasConverged()) {
    //   // RCLCPP_INFO(this->get_logger(), "ICP Converged");
    //   Eigen::Matrix4d registered_cloud_transform = icp_.getFinalTransformation().cast<double>();
    //   // std::cout << registered_cloud_transform << std::endl;
    //   pcl::transformPointCloud(*new_point_cloud_, *registered_cloud, registered_cloud_transform);
    //   *point_cloud_map_ += *registered_cloud;

    // } else {
    //   RCLCPP_WARN(this->get_logger(), "ICP Failed Convergence");
    // }
    new_data_received_ = false;
  }

  // Downsample
  // RCLCPP_INFO(this->get_logger(), "Number of points before downsample: %lu", point_cloud_map_->size());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(point_cloud_map_);
  voxel_grid.setLeafSize(voxel_filter_size_, voxel_filter_size_, voxel_filter_size_);
  voxel_grid.filter(*point_cloud_map_);
  // RCLCPP_INFO(this->get_logger(), "Number of points after downsample: %lu", point_cloud_map_->size());

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
    pcl::transformPointCloud(*new_point_cloud_, *point_cloud_map_, tf2::transformToEigen(tf_).matrix().cast<float>());
    // first_pc = true;
  } else {
    new_data_received_ = true;
  }
}

}  // namespace mapping
}  // namespace cg
