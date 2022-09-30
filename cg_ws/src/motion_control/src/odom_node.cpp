#include "motion_control/odom_node.hpp"
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

namespace cg {
namespace odom {

OdomNode::OdomNode() : Node("odom_node") {
  // Initialize publishers and subscribers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/encoder/odom", 1
  );
  act_state_pub_ = this->create_publisher<cg_msgs::msg::ActuatorState>(
    "/actuator/state", 1
  );
  enc_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
    "/encoder_telemetry", 1, std::bind(&OdomNode::odomCallback, this, std::placeholders::_1)
  );
  // Load parameters 
  this->declare_parameter<double>("half_wheel_base_m", 0.2775);
  this->get_parameter("half_wheel_base_m", half_wheel_base_m_);
  this->declare_parameter<double>("qp_steer_to_radian", 0.000099223014335);
  this->get_parameter("qp_steer_to_radian", qp_steer_to_radian_);
  this->declare_parameter<double>("qp_drive_to_pos_m", 0.0008298755187);
  this->get_parameter("qp_drive_to_pos_m", qp_drive_to_pos_m_);
  this->declare_parameter<double>("qpps_drive_to_speed_ms", 0.0008298755187);
  this->get_parameter("qpps_drive_to_speed_ms", qpps_drive_to_speed_ms_);
  this->declare_parameter<double>("qp_tool_to_fs", -0.00000666933);
  this->get_parameter("qp_tool_to_fs", qp_tool_to_fs_);

  // position covariance
  this->declare_parameter<double>("pose_cov_x", 1);
  this->get_parameter("pose_cov_x", pose_cov_x_);
  this->declare_parameter<double>("pose_cov_y", 1);
  this->get_parameter("pose_cov_y", pose_cov_y_);
  this->declare_parameter<double>("pose_cov_z", 1);
  this->get_parameter("pose_cov_z", pose_cov_z_);
  this->declare_parameter<double>("pose_cov_r", 1);
  this->get_parameter("pose_cov_r", pose_cov_r_);
  this->declare_parameter<double>("pose_cov_p", 1);
  this->get_parameter("pose_cov_p", pose_cov_p_);
  this->declare_parameter<double>("pose_cov_yaw", 1);
  this->get_parameter("pose_cov_yaw", pose_cov_yaw_);

  // velocity covariance
  this->declare_parameter<double>("twist_cov_x", 1);
  this->get_parameter("twist_cov_x", twist_cov_x_);
  this->declare_parameter<double>("twist_cov_y", 1);
  this->get_parameter("twist_cov_y", twist_cov_y_);
  this->declare_parameter<double>("twist_cov_z", 1);
  this->get_parameter("twist_cov_z", twist_cov_z_);
  this->declare_parameter<double>("twist_cov_r", 1);
  this->get_parameter("twist_cov_r", twist_cov_r_);
  this->declare_parameter<double>("twist_cov_p", 1);
  this->get_parameter("twist_cov_p", twist_cov_p_);
  this->declare_parameter<double>("twist_cov_yaw", 1);
  this->get_parameter("twist_cov_yaw", twist_cov_yaw_);

  // INITALIZE VARIABLES
  prev_x_ = 0;
  prev_y_ = 0;
  prev_heading_ = 0;

  // Set up frames for odom and base_link
  odom_msg_.child_frame_id = "base_link";
  odom_msg_.header.frame_id = "odom";

  tlast_ = this->get_clock()->now().seconds();
}

void OdomNode::odomCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg) {

  float pos_delta = -qp_drive_to_pos_m_*(msg->drive_delta_front - msg->drive_delta_rear)/2; // flip if the direction is wrong
  float steer_angle = -qp_steer_to_radian_*(msg->steer_pos_rear - msg->steer_pos_front)/2; // flip this sign if the steer is reversed 
  float drive_velocity = -qpps_drive_to_speed_ms_*(msg->drive_vel_front - msg->drive_vel_rear)/2; // flip this sign if the drive velocity is reversed

  // Actuator State message
  auto act_state_msg = cg_msgs::msg::ActuatorState();
  act_state_msg.header = msg->header;
  act_state_msg.wheel_velocity = drive_velocity;
  act_state_msg.steer_position = steer_angle;
  act_state_msg.tool_position = msg->tool_pos * qp_tool_to_fs_;
  act_state_pub_->publish(act_state_msg);

  delta_t_ = (msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9)  - tlast_;

  // vel
  odom_msg_.twist.twist.linear.x = drive_velocity;
  odom_msg_.twist.twist.linear.y = 0;
  odom_msg_.twist.twist.linear.z = 0;

  // omega
  odom_msg_.twist.twist.angular.x = 0;
  odom_msg_.twist.twist.angular.y = 0;
  odom_msg_.twist.twist.angular.z = drive_velocity*sin(steer_angle)/half_wheel_base_m_;

  // TWIST COVARIANCE
  odom_msg_.twist.covariance[0] = twist_cov_x_;
  odom_msg_.twist.covariance[7] = twist_cov_y_;
  odom_msg_.twist.covariance[14] = twist_cov_z_; 
  odom_msg_.twist.covariance[21] = twist_cov_r_;
  odom_msg_.twist.covariance[28] = twist_cov_p_;
  odom_msg_.twist.covariance[35] = twist_cov_yaw_;  

  // pos
  odom_msg_.pose.pose.position.x = prev_x_ + (pos_delta * cos(steer_angle) * cos(prev_heading_));
  odom_msg_.pose.pose.position.y = prev_y_ + (pos_delta * cos(steer_angle) * sin(prev_heading_));
  odom_msg_.pose.pose.position.z = 0;

  // rpy
  tf2::Quaternion q;
  prev_heading_ += (delta_t_ * odom_msg_.twist.twist.angular.z);
  q.setRPY(0, 0, prev_heading_);
  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  odom_msg_.pose.pose.orientation.w = q.w();

  // POSE COVARIANCE
  odom_msg_.pose.covariance[0] = pose_cov_x_;
  odom_msg_.pose.covariance[7] = pose_cov_y_;
  odom_msg_.pose.covariance[14] = pose_cov_z_; 
  odom_msg_.pose.covariance[21] = pose_cov_r_;
  odom_msg_.pose.covariance[28] = pose_cov_p_;
  odom_msg_.pose.covariance[35] = pose_cov_yaw_;   

  prev_x_ = odom_msg_.pose.pose.position.x;
  prev_y_ = odom_msg_.pose.pose.position.y;

  odom_msg_.header.stamp = this->get_clock()->now();

  tlast_ = msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9;

  odom_pub_->publish(odom_msg_);
}

} // namespace odom
} // namespace cg
