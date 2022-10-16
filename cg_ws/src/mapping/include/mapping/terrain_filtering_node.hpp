#ifndef MAPPING__TERRAIN_FILTER_NODE_HPP
#define MAPPING__TERRAIN_FILTER_NODE_HPP

// TODO: FIND WHICH PACKAGES ARE ACTUALLY NEEDED
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// includes for the box filters
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace cg {
namespace mapping {

class TerrainFilteringNode : public rclcpp::Node {

public:
  TerrainFilteringNode();

  /* Create Buffer and TF Listener PTR */
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  /* Variables */
  tf2::Stamped<tf2::Transform> transform_;
  std::string target_frame_;
  std::string source_frame_;
  sensor_msgs::msg::PointCloud2 cloud_in_, cloud_out_;
  sensor_msgs::msg::PointCloud2 box_cloud_out_;
  pcl::CropBox<pcl::PointXYZ> crop;

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_points_sub_;

  /* Callbacks */
  void rawPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /* Parameters */
  bool use_sor_;
  int sor_mean_k_;
  double sor_stddev_mul_thresh_;
};


}  // namespace mapping
}  // namespace cg

#endif  // MAPPING__TERRAIN_FILTER_NODE_HPP
