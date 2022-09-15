#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mapping/site_map.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
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
  rclcpp::TimerBase::SharedPtr timer_; // For looping publish

  // rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr telem_sub_;

  /* Callbacks */
  void newPtsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timerCallback(); // For looping publish
  // void telemCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);


  /* Variables */
  cg::mapping::SiteMap siteMap_;
  // float driveSpeed = 0.0f;

  /* Parameters */
  int height_;
  int width_; 
  float resolution_;

}; // class SiteMapNode

} // namespace mapping
} // namespace cg
