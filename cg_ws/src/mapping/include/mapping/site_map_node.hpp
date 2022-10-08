#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mapping/site_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <cg_msgs/srv/site_map.hpp> // Service for sending SiteMap height data
// #include <cg_msgs/msg/encoder_telemetry.hpp>


namespace cg {
namespace mapping {

class SiteMapNode : public rclcpp::Node {

public:
  SiteMapNode();

private: 
  /* Publishers and Subscribers */ 
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr new_points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_pub_;
  rclcpp::TimerBase::SharedPtr PubTimer_; // For looping publish
  rclcpp::TimerBase::SharedPtr SiteNormalizeTimer_; // For looping publish

  /* Services */
  rclcpp::Service<cg_msgs::srv::SiteMap>::SharedPtr server_;
  void sendSiteMap(cg_msgs::srv::SiteMap::Request::SharedPtr req, cg_msgs::srv::SiteMap::Response::SharedPtr res);

  // rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr telem_sub_;

  /* Callbacks */
  void newPtsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void PubTimerCallback(); // For looping publish
  void SiteNormalizeTimerCallback(); // For looping publish

  /* Variables */
  cg::mapping::SiteMap siteMap_;

  /* Parameters */
  int height_;
  int width_; 
  float resolution_;
  float filterMaxTerrain_;
  float filterMinTerrain_;
  float xTransform_;
  float yTransform_;
  float unseenGridHeight_;
  float incomingPointVariance_;
  float cellStartingVariance_;
  float minCellVariance_;

}; // class SiteMapNode

} // namespace mapping
} // namespace cg
