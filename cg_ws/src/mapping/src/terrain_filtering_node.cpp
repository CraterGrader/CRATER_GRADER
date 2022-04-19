#include "mapping/terrain_filtering_node.hpp"

// TODO DELETE THESE INCLUDES -- TEMPORARY FOR PLANE REMOVAL TEST
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Includes for outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/common/common.h>

namespace cg {
namespace mapping {

TerrainFilteringNode::TerrainFilteringNode() : Node("terrain_filtering_node"){
  // create ptr to listener 
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Parameters 
  // TODO: add to a parameters file
  source_frame_ = "realsense_frame";
  target_frame_ = "map";

  // Statistical Outlier Removal Params
  this->declare_parameter<bool>("use_sor", true);
  this->get_parameter("use_sor", use_sor_);
  this->declare_parameter<int>("sor_mean_k", 50);
  this->get_parameter("sor_mean_k", sor_mean_k_);
  this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
  this->get_parameter("sor_stddev_mul_thresh", sor_stddev_mul_thresh_);

  // Plane Segmentation Params
  this->declare_parameter<bool>("use_plane_seg", true);
  this->get_parameter("use_plane_seg", use_plane_seg_);
  this->declare_parameter<double>("plane_seg_dist_thresh", 0.03);
  this->get_parameter("plane_seg_dist_thresh", plane_seg_dist_thresh_);

  // Initialize publishers and subscribers
  filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1
  );
  plane_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered_plane", 1
  );
  below_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered_below", 1
  );
  above_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered_above", 1
  );
  raw_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&TerrainFilteringNode::rawPointsCallback, this, std::placeholders::_1)
  );
}

void TerrainFilteringNode::rawPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tfBuffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    cloud_in_ = *msg;
    tf2::doTransform(cloud_in_, cloud_out_, transformStamped);
    cloud_out_.header.frame_id = target_frame_;

    // convert sensor_msgs::msg::PointCloud2::SharedPtr to pcl::PointCloud<pcl::PointXYZ>
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_out_,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    // RCLCPP_INFO(this->get_logger(), "SIZE: %d", temp_cloud->size());

    // setup output cloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Declare instance of filter:
    pcl::CropBox<pcl::PointXYZ> crop;

    // Set input:
    crop.setInputCloud(temp_cloud);

    // Set parameters
    Eigen::Vector4f min_point = Eigen::Vector4f(1.0, 1.0, -10.0, 0);
    Eigen::Vector4f max_point = Eigen::Vector4f(6.0, 6.0, 10.0, 0);
    crop.setMin(min_point);
    crop.setMax(max_point);

    // Filter
    crop.filter(*xyz_filtered_cloud);

    // Statistical outlier removal for salt-and-pepper noise
    // https://pcl.readthedocs.io/en/latest/statistical_outlier.html
    if (use_sor_) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(xyz_filtered_cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_mul_thresh_);
      sor.filter(*xyz_filtered_cloud);
    }

    // Plane removal
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
    seg.setDistanceThreshold (plane_seg_dist_thresh_);

    seg.setInputCloud (xyz_filtered_cloud);
    seg.segment (*inliers, *coefficients);
    // for (const auto & c: coefficients->values) {
    //   std::cout << c << ',' << ' ';
    // }
    bool is_plane_facing_up = ((coefficients->values.size() == 4) && (coefficients->values[3] > 0));
    // std::cout << is_plane_facing_up << std::endl;
    if (use_plane_seg_) {
      if (inliers->indices.size () == 0) {
        RCLCPP_WARN (this->get_logger(), "Could not estimate a planar model for the given dataset.");
      } else {
        // https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(xyz_filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*xyz_filtered_cloud);
      }
    }

    // Test filter out above/below ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(xyz_filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*plane);
    pcl::ConvexHull<pcl::PointXYZ> hull;
		hull.setInputCloud(plane);
    hull.setDimension(2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*convexHull);
    pcl::PointCloud<pcl::PointXYZ>::Ptr below_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr above_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (hull.getDimension()==2) {
      pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
      // Below ground points
			prism.setInputCloud(xyz_filtered_cloud);
			prism.setInputPlanarHull(convexHull);
      // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
			// Second parameter: maximum Z value
      // if (is_plane_facing_up) {
      prism.setHeightLimits(-0.03f, 10.0f);
      // } else {
        // prism.setHeightLimits(-10.0f, 0.03f);
      // }
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
      prism.segment(*objectIndices);

      extract.setIndices(objectIndices);
      extract.setNegative(true);
      extract.filter(*below_ground_cloud);

      // Above ground points
			prism.setInputCloud(xyz_filtered_cloud);
			prism.setInputPlanarHull(convexHull);
      // if (is_plane_facing_up) {
      prism.setHeightLimits(-10.0f, 0.03f);
      // } else {
        // prism.setHeightLimits(-0.03f, 10.0f);
      // }
      prism.segment(*objectIndices);
      extract.setIndices(objectIndices);
      extract.setNegative(true);
      extract.filter(*above_ground_cloud);

      // TODO size checks

      float avg_above_ground_z = 0;
      for (const auto & pt : above_ground_cloud->points) {
        avg_above_ground_z += pt.z / above_ground_cloud->points.size();
      }
      float avg_below_ground_z = 0;
      for (const auto & pt : below_ground_cloud->points) {
        avg_below_ground_z += pt.z / below_ground_cloud->points.size();
      }
      float avg_plane_z = 0;
      for (const auto & pt : plane->points) {
        avg_plane_z += pt.z / plane->points.size();
      }
      // float below_ground_min_z = below_ground_cloud->points[0].z, below_ground_max_z = above_ground_cloud->points[0].z;
      // RCLCPP_INFO(this->get_logger(), "Avg Above/Plane/Below Ground Z: %f, %f, %f", avg_above_ground_z, avg_plane_z, avg_below_ground_z);
      if (avg_plane_z > avg_above_ground_z || avg_plane_z < avg_below_ground_z) {
        RCLCPP_INFO(this->get_logger(), "Flipped Z");
        auto temp_cloud = above_ground_cloud;
        above_ground_cloud = below_ground_cloud;
        below_ground_cloud = temp_cloud;
      }
      // pcl::PointXYZ above_ground_min, above_ground_max;
      // pcl::PointXYZ below_ground_min, below_ground_max;
      // pcl::PointXYZ plane_min, plane_max;
      // above_ground_cloud->is_dense = false;
      // below_ground_cloud->is_dense = false;
      // pcl::getMinMax3D(*above_ground_cloud, above_ground_min, above_ground_max);
      // pcl::getMinMax3D(*below_ground_cloud, below_ground_min, below_ground_max);
      // pcl::getMinMax3D(*plane, plane_min, plane_max);
      // RCLCPP_INFO(this->get_logger(), "Above Ground Min: %f,%f,%f", above_ground_min.x, above_ground_min.y, above_ground_min.z);
      // RCLCPP_INFO(this->get_logger(), "Plane Ground Min: %f,%f,%f", plane_min.x, plane_min.y, plane_min.z);
      // RCLCPP_INFO(this->get_logger(), "Below Ground Min: %f,%f,%f", below_ground_min.x, below_ground_min.y, below_ground_min.z);
      // RCLCPP_INFO(this->get_logger(), "Above Ground Max: %f,%f,%f", above_ground_max.x, above_ground_max.y, above_ground_max.z);
      // RCLCPP_INFO(this->get_logger(), "Plane Ground Max: %f,%f,%f", plane_max.x, plane_max.y, plane_max.z);
      // RCLCPP_INFO(this->get_logger(), "Below Ground Max: %f,%f,%f", below_ground_max.x, below_ground_max.y, below_ground_max.z);
      // std::cout << above_ground_max.z << ',' << below_ground_max.z << '\n';
      // if (above_ground_max.z < below_ground_max.z) {
      //   RCLCPP_INFO(this->get_logger(), "Flipped Z");
      // }
    }

    // Convert from pcl::PointCloud<T> back to sensor_msgs::PointCloud2
    pcl::toROSMsg(*xyz_filtered_cloud, box_cloud_out_);

    // Publish
    filtered_points_pub_->publish(box_cloud_out_);
    pcl::toROSMsg(*below_ground_cloud, box_cloud_out_);
    below_points_pub_->publish(box_cloud_out_);
    pcl::toROSMsg(*above_ground_cloud, box_cloud_out_);
    above_points_pub_->publish(box_cloud_out_);
    pcl::toROSMsg(*plane, box_cloud_out_);
    plane_points_pub_->publish(box_cloud_out_);
    } 
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),"%s", ex.what());
    }
  
}

}  // namespace mapping
}  // namespace cg
