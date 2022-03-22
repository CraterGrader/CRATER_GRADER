#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>

namespace cg {
namespace odom {

class OdomNode : public rclcpp::Node {

public:
  OdomNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr enc_sub_;

  /* Message data */
  nav_msgs::msg::Odometry odom_msg_;

  /* Callbacks */
  void odomCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);

  /* Variables */
  double prev_x_;
  double prev_y_;
  double prev_heading_;
  double delta_t_;
  double tlast_;
  double tcurr_;

} // class OdomNode

} // namespace odom
} // namespace cg
