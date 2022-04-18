#pragma once 

#include<chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cg_msgs/msg/slip.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
#include <cmath> // std::abs
#include <algorithm> // std::max
#include <list> // moving average filter

namespace cg {
namespace slip {

class SlipEstimateNode : public rclcpp::Node {

public:
  SlipEstimateNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::Slip>::SharedPtr slip_pub_;
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr enc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_sub_;
  rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr telem_sub_;

  /* Message data */
  cg_msgs::msg::Slip slip_msg_;
  float last_global_vel_;
  float last_enc_vel_;
  bool enc_init = false;

  // Raw slip calculation
  float actual_front_vel_;
  float actual_rear_vel_;
  float nonzero_slip_thresh_;
  float curr_raw_slip_;

  // Moving average filter
  int filter_window_;
  std::list<float> slip_window_;
  float curr_slip_avg_;

  // Velocity estimate
  bool global_init_ = true;
  float curr_vel_estimate_;
  float vel_twst_;
  float last_x_;
  float last_y_;
  double last_t_;
  double delta_t_;
  int sampler_ = 0;
  int sampling_steps_ = 20;

  float slip_latch_thresh_ = 0.7;
  float slip_velocity_latch_release_ = 2000;
  bool slip_latch_;

  // Velocity estimate moving average filter
  int vel_filter_window_ = 3;
  std::list<float> vel_window_;
  float curr_vel_avg_;
  // float curr_vel_avg_;

  /* Callbacks */
  void slipCallback();
  void globalCallback(const nav_msgs::msg::Odometry::SharedPtr global_msg);
  void encCallback(const nav_msgs::msg::Odometry::SharedPtr enc_msg);
  void slipTelemetryCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);

  // rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(50),
  //   std::bind(&SlipEstimateNode::slipCallback, this));

  /* Helpers */
  void updateMovingAverage();
  float updateMovingAverage(std::list<float> &list, float &new_val, int window_size);

  // Syncronize messages before processing

}; // class SlipEstimateNode

} // namespace slip
} // namespace cg
