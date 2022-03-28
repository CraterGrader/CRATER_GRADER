#pragma once 

#include<chrono>
#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/slip.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cg {
namespace slip {

class SlipEstimateNode : public rclcpp::Node {

public:
  SlipEstimateNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::Slip>::SharedPtr slip_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr enc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  /* Message data */
  cg_msgs::msg::Slip slip_msg_;
  float last_odom_vel_;
  bool odom_init = false;
  float last_enc_vel_;
  bool enc_init = false;

  /* Callbacks */
  void slipCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr& enc_msg,
      const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void encCallback(const nav_msgs::msg::Odometry::SharedPtr enc_msg);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SlipEstimateNode::slipCallback, this));

  // Syncronize messages before processing

}; // class SlipEstimateNode

} // namespace slip
} // namespace cg
