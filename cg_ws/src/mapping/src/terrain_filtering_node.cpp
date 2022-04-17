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
    bool is_plane_facing_up = ((coefficients->values.size() == 4) && (coefficients->values[2] > 0));
    // std::cout << std::endl;
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
      if (is_plane_facing_up) {
        prism.setHeightLimits(-0.03f, 10.0f);
      } else {
        prism.setHeightLimits(-10.0f, 0.03f);
      }
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
      prism.segment(*objectIndices);

      extract.setIndices(objectIndices);
      extract.setNegative(true);
      extract.filter(*below_ground_cloud);

      // Above ground points
			prism.setInputCloud(xyz_filtered_cloud);
			prism.setInputPlanarHull(convexHull);
      if (is_plane_facing_up) {
        prism.setHeightLimits(-10.0f, 0.03f);
      } else {
        prism.setHeightLimits(-0.03f, 10.0f);
      }
      prism.segment(*objectIndices);
      extract.setIndices(objectIndices);
      extract.setNegative(true);
      extract.filter(*above_ground_cloud);
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
