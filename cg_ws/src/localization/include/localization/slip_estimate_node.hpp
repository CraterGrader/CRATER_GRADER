#pragma once 

#include<chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cg_msgs/msg/slip.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
#include <cg_msgs/msg/actuator_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cmath> // std::abs
#include <algorithm> // std::max
#include <list> // moving average filter
#include "localization/kalman_filter_linear.hpp"

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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_avg_sub_;

  rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr telem_sub_;

  rclcpp::Subscription<cg_msgs::msg::ActuatorState>::SharedPtr act_sub_;

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish

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

  // Velocity estimate
  bool global_init_ = true;
  float curr_vel_estimate_;
  float vel_twst_;
  float last_x_;
  float last_y_;
  double last_t_;
  double delta_t_;
  int sampler_ = 0;
  int sampling_steps_ = 12;

  // Velocity estimate moving average filter
  int vel_filter_window_ = 3;
  std::list<float> vel_window_;
  float curr_vel_avg_;
  // float curr_vel_avg_;

  /* Good params */
  float vel_wheels_ = 0.; // Wheel velocity averaged between front and back
  float curr_slip_;
  float nonzero_slip_thresh_wheel_ms_;
  float slip_latch_thresh_ms_;
  float slip_velocity_latch_release_ms_;
  float nonzero_slip_thresh_vehicle_ms_;

  int slip_window_size_ = 10;
  std::list<float> slip_window_;
  float slip_avg_;

  int vel_kf_window_size_ = 10;
  std::list<float> vel_kf_window_;
  float vel_kf_avg_;

  float slip_latch_thresh_ = 0.8;
  float slip_velocity_latch_release_ = 2000;
  bool slip_latch_;

  // Kalman filter for velocity estimation
  int kf_n_ = 4; // Number of states
  int kf_m_ = 2;            // Number of measurements
  double kf_dt_ = 0.05; // Time step, should be ~hz of callback
  float vel_kf_ = 0.;       // Estimated velocity from Kalman Filter

  float Qx_;
  float Qxdot_;
  float Qy_;
  float Qydot_;
  float Rx_;
  float Ry_;
  float Px_;
  float Pxdot_;
  float Py_;
  float Pydot_;

  Eigen::VectorXd z_;
  Eigen::VectorXd xhat_;
  Eigen::VectorXd x0_;

  cg::localization::KalmanFilterLinear kf_vel_;

  /* Callbacks */
  void slipCallback();
  void globalCallback(const nav_msgs::msg::Odometry::SharedPtr global_msg);
  void encCallback(const nav_msgs::msg::Odometry::SharedPtr enc_msg);
  void actStateCallback(const cg_msgs::msg::ActuatorState::SharedPtr msg);
  void slipTelemetryCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);
  void timerCallback(); // For looping publish
  void uwbAvgCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

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
