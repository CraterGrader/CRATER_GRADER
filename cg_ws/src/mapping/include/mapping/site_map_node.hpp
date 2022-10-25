#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mapping/site_map.hpp>
#include <mapping/map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <cg_msgs/srv/site_map.hpp> // Service for sending SiteMap height data
#include <cg_msgs/srv/save_map.hpp> // Service for saving SiteMap height data to csv file

namespace cg {
namespace mapping {

class SiteMapNode : public rclcpp::Node {

public:
  SiteMapNode();

private: 
  /* Publishers and Subscribers */ 
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr new_points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_seen_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_variance_map_pub_;

  // timer
  rclcpp::TimerBase::SharedPtr viz_timer_;

  /* Services */
  rclcpp::Service<cg_msgs::srv::SiteMap>::SharedPtr site_map_server_;
  rclcpp::Service<cg_msgs::srv::SaveMap>::SharedPtr save_map_server_;
  void sendSiteMap(cg_msgs::srv::SiteMap::Request::SharedPtr req, cg_msgs::srv::SiteMap::Response::SharedPtr res);
  void saveMap(cg_msgs::srv::SaveMap::Request::SharedPtr req, cg_msgs::srv::SaveMap::Response::SharedPtr res);

  /* Callbacks */
  void new_pts_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void map_viz_callback();

  /* Methods */

  /* Variables */
  cg::mapping::SiteMap siteMap_;
  cg::mapping::Map<float> fileMap_; // height map to use for saving siteMap_ data to a file (probably best to use inheritence but some refactor needed)

  /* Parameters */
  int height_;
  int width_; 
  float resolution_;
  float xTransform_;
  float yTransform_;
  float unseenGridHeight_;
  float incomingPointVariance_;
  float cellStartingVariance_;
  float minCellVariance_;

}; // class SiteMapNode

} // namespace mapping
} // namespace cg
