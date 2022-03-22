#include "motion_control/odom_node.hpp"
#include <math.h>

namespace cg {
namespace odom {

OdomNode::OdomNode() : Node("odom_node") {
  // Initialize publishers and subscribers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/encoder/odom", 1
  );
  enc_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
    "/encoder_telemetry", 1, std::bind(&OdomNode::odomCallback, this, std::placeholders::_1)
  );
  // Load parameters 
  this->declare_parameter<int>("half_wheel_base_m", 0.2775);
  this->get_parameter("half_wheel_base_m", half_wheel_base_m_);
  this->declare_parameter<int>("qp_steer_to_radian", 0.000099223014335);
  this->get_parameter("qp_steer_to_radian", qp_steer_to_radian_);
  this->declare_parameter<int>("qp_drive_to_pos_m", 0.0008298755187);
  this->get_parameter("qp_drive_to_pos_m", qp_drive_to_pos_m_);
  this->declare_parameter<int>("qpps_drive_to_speed_ms", 0.0008298755187);
  this->get_parameter("qpps_drive_to_speed_ms", qpps_drive_to_speed_ms_);

  // TODO: INITALIZE VARIABLES
  prev_x_ = 0;
  prev_y_ = 0;
  prev_heading_ = 0;
  tlast_ = 0;
  tcurr_ = 0;
}

void OdomNode::odomCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg) {

  data = msg->data;

  float pos_delta = qp_drive_to_pos_m_*(data.drive_delta_front-data.drive_delta_rear)/2; // flip if the direction is wrong
  float steer_angle = qp_steer_to_radian_*(data.steer_pos_rear-data.steer_pos_front)/2; // flip this sign if the steer is reversed 
  float drive_velocity = qpps_drive_to_speed_ms_*(data.drive_vel_front-data.drive_vel_rear)/2; // flip this sign if the drive velocity is reversed 

  t_curr_ = data.header.stamp;
  delta_t_ = t_curr_ - t_last_;

  // TODO: ASK RUSSELL about odom_msg_.header.frame_id and odom_msg_.child_frame_id

  // vel
  odom_msg_.twist.twist.linear.x = drive_velocity*cos(prev_heading_+steer_angle);
  odom_msg_.twist.twist.linear.y = drive_velocity*sin(prev_heading_+steer_angle);
  odom_msg_.twist.twist.linear.z = 0;

  // omega
  odom_msg_.twist.twist.angular.x = 0;
  odom_msg_.twist.twist.angular.y = 0;
  odom_msg_.twist.twist.angular.z = drive_velocity*cos(steer_angle)/half_wheel_base_m_;

  // parameter
  // TODO: find a way to display covariance
  // odom_msg_.twist.covariance = float64[36] covariance

  // pos
  odom_msg_.pose.pose.position.x = prev_x_ + (pos_delta * cos(prev_heading_+steer_angle));
  odom_msg_.pose.pose.position.y = prev_y_ + (pos_delta * sin(prev_heading_+steer_angle));
  odom_msg_.pose.pose.position.z = 0;

  // rpy
  odom_msg_.pose.pose.quaternion.x = 0;
  odom_msg_.pose.pose.quaternion.y = 0;
  odom_msg_.pose.pose.quaternion.z = prev_heading_ + (delta_t_*odom_msg_.twist.twist.angular.z);
  odom_msg_.pose.pose.quaternion.w = 1 ;

  // parameter
  // TODO: find a way to display covariance
  // odom_msg_.pose.covariance = float64[36] covariance

  prev_x_ = odom_msg_.pose.pose.position.x;
  prev_y_ = odom_msg_.pose.pose.position.y;
  prev_heading_ = odom_msg_.pose.pose.quaternion.z;

  odom_msg_.header.stamp = this->get_clock()->now();

  t_last_ = data.header.stamp;

  odom_pub_->publish(odom_msg_);
}

} // namespace odom
} // namespace cg
