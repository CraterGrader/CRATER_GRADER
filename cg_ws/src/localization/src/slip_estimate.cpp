#include "localization/slip_estimate.hpp"
#include<math.h>

namespace cg {
namespace slip {

SlipEstimateNode::SlipEstimateNode() : Node("slip_estimate_node") {
  
  // Initialize publishers and subscribers
  slip_pub_ = this->create_publisher<std_msgs::Float64>(
    "/slip_estimate", 1
  );
  
  enc_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/encoder/odom", 1, std::bind(&SlipEstimateNode::encCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered/ekf_odom_node", 1, std::bind(&SlipEstimateNode::odomCallback, this, std::placeholders::_1)
  );

}


void SlipEstimateNode::slipCallback() {

  // If either are not initialized, can't calculate slip
  if (!odom_init || !enc_init) return;

  slip_msg_.header.stamp = this->get_clock()->now();

  // Slip is percentage difference between encoder-read velocty and fused odometry velocity.
  slip.slip = 100 * (last_enc_vel - last_odom_vel_) / last_odom_vel;

  slip_pub_->publish(slip_msg_);

}


void SlipEstimateNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr odom_msg) {

  last_odom_vel_ = sqrt(
    odom_msg.twist.twist.linear.x**2 + 
    odom_msg.twist.twist.linear.y**2);

  odom_init = true;

}


void SlipEstimateNode::encCallback(
  const nav_msgs::msg::Odometry::SharedPtr enc_msg) {

  last_enc_vel_ = sqrt(
    enc_msg.twist.twist.linear.x**2 + 
    enc_msg.twist.twist.linear.y**2);

  enc_init = true;

}

} // namespace odom
} // namespace cg
