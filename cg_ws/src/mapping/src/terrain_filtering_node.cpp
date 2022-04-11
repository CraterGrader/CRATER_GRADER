#include "mapping/terrain_filtering_node.hpp"

// TODO DELETE THESE INCLUDES -- TEMPORARY FOR PLANE REMOVAL TEST
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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

  // // Initialize publishers and subscribers
  filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1
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

    // TEMPORARY PLANE REMOVAL -- TODO DELETE
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
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (xyz_filtered_cloud);
    seg.segment (*inliers, *coefficients);

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

    // Convert from pcl::PointCloud<T> back to sensor_msgs::PointCloud2
    pcl::toROSMsg(*xyz_filtered_cloud, box_cloud_out_);

    // Publish
    filtered_points_pub_->publish(box_cloud_out_);
    } 
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),"%s", ex.what());
    }
  
}

}  // namespace mapping
}  // namespace cg
