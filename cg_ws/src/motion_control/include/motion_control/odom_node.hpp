#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cg_msgs/msg/actuator_state.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>

namespace cg {
namespace odom {

class OdomNode : public rclcpp::Node {

public:
  OdomNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<cg_msgs::msg::ActuatorState>::SharedPtr act_state_pub_;
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

  /* Parameters */  
  double half_wheel_base_m_;
  double qp_steer_to_radian_;
  double qp_drive_to_pos_m_;
  double qpps_drive_to_speed_ms_;
  double qp_tool_to_fs_;

  double pose_cov_x_;
  double pose_cov_y_;
  double pose_cov_z_;
  double pose_cov_r_;
  double pose_cov_p_;
  double pose_cov_yaw_;
  double twist_cov_x_;
  double twist_cov_y_;
  double twist_cov_z_;
  double twist_cov_r_;
  double twist_cov_p_;
  double twist_cov_yaw_;

}; // class OdomNode

} // namespace odom
} // namespace cg
