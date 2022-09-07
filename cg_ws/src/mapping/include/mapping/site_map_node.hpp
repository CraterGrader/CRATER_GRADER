#pragma once 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mapping/site_map.hpp>

// hail marry
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>

namespace cg {
namespace mapping {

class SiteMapNode : public rclcpp::Node {

public:
  SiteMapNode();

private: 
  /* Publishers and Subscribers */ 
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr new_points_sub_;

  /* Callbacks */
  void newPtsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  /* Variables */
  cg::mapping::SiteMap siteMap_;

  /* Parameters */

}; // class SiteMapNode

} // namespace mapping
} // namespace cg
