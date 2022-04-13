#include "localization/slip_estimate_node.hpp"
#include<math.h>


namespace cg {
namespace slip {

SlipEstimateNode::SlipEstimateNode() : Node("slip_estimate_node") {
  
  // Initialize publishers and subscribers
  slip_pub_ = this->create_publisher<cg_msgs::msg::Slip>(
    "/slip_estimate", 1
  );
  
  // enc_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/encoder/odom", 1, std::bind(&SlipEstimateNode::encCallback, this, std::placeholders::_1)
  // );

  // global_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/odometry/filtered/ekf_global_node", 1, std::bind(&SlipEstimateNode::globalCallback, this, std::placeholders::_1)
  // );

  telem_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
    "/encoder_telemetry", 1, std::bind(&SlipEstimateNode::slipTelemetryCallback, this, std::placeholders::_1)
  );

  // Load parameters
  this->declare_parameter<float>("nonzero_slip_thresh", 10.0);
  this->get_parameter("nonzero_slip_thresh", nonzero_slip_thresh_);
  this->declare_parameter<int>("filter_window", 10);
  this->get_parameter("filter_window", filter_window_);
}

void SlipEstimateNode::updateMovingAverage(){

  // Build up the filter
  slip_window_.push_front(curr_raw_slip_);
  if (static_cast<int>(slip_window_.size()) > filter_window_) {
    slip_window_.pop_back(); // remove oldest value
  }

  // Update the filter
  curr_slip_avg_ = std::accumulate(slip_window_.begin(), slip_window_.end(), 0.0) / slip_window_.size();
}

void SlipEstimateNode::slipTelemetryCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg) {

  // Calculate current raw slip value
  actual_front_vel_ = static_cast<float>(std::abs(msg->drive_vel_rear));
  actual_rear_vel_ = static_cast<float>(std::abs(msg->drive_vel_front));
  
  if (actual_rear_vel_ > nonzero_slip_thresh_) 
  {
    // Calculate slip if it's above the calculation threshold
    // Expect 0 for no slip, 1 for 100% slip, clamp at zero
    curr_raw_slip_ = std::max(static_cast<float>(0.0), (actual_rear_vel_ - actual_front_vel_) / actual_rear_vel_);
  }
  else {
    curr_raw_slip_ = static_cast<float>(0.0);
  }

  // Filter slip estimate with moving average
  updateMovingAverage(); // call to update curr_slip_avg_

  // Publish the slip estimate
  slip_msg_.slip = curr_slip_avg_;
  slip_msg_.header.stamp = this->get_clock()->now();
  slip_pub_->publish(slip_msg_);

  // Slip is percentage difference between encoder-read velocty and fused odometry velocity.
}


// void SlipEstimateNode::slipCallback() {

//   // If either are not initialized, can't calculate slip
//   if (!global_init || !enc_init) return;

//   slip_msg_.header.stamp = this->get_clock()->now();

//   if (last_enc_vel_ != 0.0) 
//   {
//     slip_msg_.slip = 100 * (last_enc_vel_ - last_global_vel_) / last_enc_vel_;
//   }
//   else {
//     slip_msg_.slip = 0;
//   }

//   slip_pub_->publish(slip_msg_);

//   // Slip is percentage difference between encoder-read velocty and fused odometry velocity.


// }


// void SlipEstimateNode::globalCallback(
//   const nav_msgs::msg::Odometry::SharedPtr global_msg) {

//   last_global_vel_ = sqrt(
//     pow(global_msg->twist.twist.linear.x, 2.0) + 
//     pow(global_msg->twist.twist.linear.y, 2.0));

//   global_init = true;

// }


// void SlipEstimateNode::encCallback(
//   const nav_msgs::msg::Odometry::SharedPtr enc_msg) {

//   last_enc_vel_ = sqrt(
//     pow(enc_msg->twist.twist.linear.x, 2.0) + 
//     pow(enc_msg->twist.twist.linear.y, 2.0));

//   enc_init = true;

// }

} // namespace odom
} // namespace cg
