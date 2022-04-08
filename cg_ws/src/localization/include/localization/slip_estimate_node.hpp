#pragma once 

#include<chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cg_msgs/msg/slip.hpp>

namespace cg {
namespace slip {

class SlipEstimateNode : public rclcpp::Node {

public:
  SlipEstimateNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::Slip>::SharedPtr slip_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr enc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_sub_;

  /* Message data */
  cg_msgs::msg::Slip slip_msg_;
  float last_global_vel_;
  bool global_init = false;
  float last_enc_vel_;
  bool enc_init = false;

  /* Callbacks */
  void slipCallback();
  void globalCallback(const nav_msgs::msg::Odometry::SharedPtr global_msg);
  void encCallback(const nav_msgs::msg::Odometry::SharedPtr enc_msg);

  rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SlipEstimateNode::slipCallback, this));

  // Syncronize messages before processing

}; // class SlipEstimateNode

} // namespace slip
} // namespace cg
