#include "localization/slip_estimate_node.hpp"
#include <math.h>

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

  global_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered/ekf_global_node", 1, std::bind(&SlipEstimateNode::globalCallback, this, std::placeholders::_1)
  );

  telem_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
    "/encoder_telemetry", 1, std::bind(&SlipEstimateNode::slipTelemetryCallback, this, std::placeholders::_1)
  );

  act_sub_ = this->create_subscription<cg_msgs::msg::ActuatorState>(
      "/actuator/state", 1, std::bind(&SlipEstimateNode::actStateCallback, this, std::placeholders::_1));

  // Timer callback
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SlipEstimateNode::timerCallback, this));

  // Load parameters
  this->declare_parameter<float>("nonzero_slip_thresh", 10.0);
  this->get_parameter("nonzero_slip_thresh", nonzero_slip_thresh_);
  this->declare_parameter<float>("nonzero_slip_thresh_ms", 0.05);
  this->get_parameter("nonzero_slip_thresh_ms", nonzero_slip_thresh_ms_);
  this->declare_parameter<float>("slip_latch_thresh_ms", 0.8);
  this->get_parameter("slip_latch_thresh_ms", slip_latch_thresh_ms_);
  this->declare_parameter<float>("slip_velocity_latch_release_ms", 0.06);
  this->get_parameter("slip_velocity_latch_release_ms", slip_velocity_latch_release_ms_);

  this->declare_parameter<float>("Qx", 10.0);
  this->get_parameter("Qx", Qx_);
  this->declare_parameter<float>("Qxdot", 10.0);
  this->get_parameter("Qxdot", Qxdot_);
  this->declare_parameter<float>("Qy", 10.0);
  this->get_parameter("Qy", Qy_);
  this->declare_parameter<float>("Qydot", 10.0);
  this->get_parameter("Qydot", Qydot_);
  this->declare_parameter<float>("Rx", 10.0);
  this->get_parameter("Rx", Rx_);
  this->declare_parameter<float>("Ry", 10.0);
  this->get_parameter("Ry", Ry_);
  this->declare_parameter<float>("Px", 10.0);
  this->get_parameter("Px", Px_);
  this->declare_parameter<float>("Pxdot", 10.0);
  this->get_parameter("Pxdot", Pxdot_);
  this->declare_parameter<float>("Py", 10.0);
  this->get_parameter("Py", Py_);
  this->declare_parameter<float>("Pydot", 10.0);
  this->get_parameter("Pydot", Pydot_);

  // Kalman Filter for velocity estimation
  // Discrete LTI projectile motion, measuring position only, constant velocity kinematics
  Eigen::MatrixXd A_(kf_n_, kf_n_); // System dynamics matrix
  Eigen::MatrixXd H_(kf_m_, kf_n_); // Measurement observation matrix
  Eigen::MatrixXd Q_(kf_n_, kf_n_); // Process noise covariance
  Eigen::MatrixXd R_(kf_m_, kf_m_); // Measurement noise covariance
  Eigen::MatrixXd P_(kf_n_, kf_n_); // Estimate error covariance

  Eigen::VectorXd x0_(kf_n_);

  // state vector x0 = [x; xdot; y; ydot]
  A_ << 1, kf_dt_, 0,   0,
        0,   1,    0,   0,
        0,   0,    1, kf_dt_,
        0,   0,    0,   1;
  H_ << 1, 0, 0, 0,
        0, 0, 1, 0;
  // Process uncertainty
  Q_ << Qx_, .0, .0, .0,
        .0, Qxdot_, .0, .0,
        .0, .0, Qy_, .0,
        .0, .0, .0, Qydot_;
  // Measurement uncertainty
  R_ << Rx_, .0,
        .0, Ry_;
  // Initially give high uncertainty to unestimated states
  P_ << Px_, .0, .0, .0,
        .0, Pxdot_, 0, .0,
        .0, 0, Py_, .0,
        .0, 0, .0, Pydot_;

  // Construct the filter
  cg::localization::KalmanFilterLinear kf_vel_tmp_(A_, H_, Q_, R_, P_);

  kf_vel_ = kf_vel_tmp_;

  // Best guess of initial states
  x0_ << 0, 0, 0, 0; // [x; xdot; y; ydot]
  kf_vel_.init(x0_);
}

void SlipEstimateNode::timerCallback()
{
  // Calculate slip estimate
  if (vel_wheels_ > nonzero_slip_thresh_ms_)
  {
    // Calculate slip if it's above the calculation threshold
    // Expect 0 for no slip, 1 for 100% slip, clamp at zero

    curr_slip_ = std::max(static_cast<float>(0.0), (vel_wheels_ - vel_kf_) / vel_wheels_);
  }
  else
  {
    curr_slip_ = static_cast<float>(0.0);
  }

  slip_avg_ = updateMovingAverage(slip_window_, curr_slip_, slip_window_size_);

  // Evaluate slip latch conditions
  if (!slip_latch_)
  {
    // Activate latch if average slip is above a threshold
    if (slip_avg_ > slip_latch_thresh_ms_)
    {
      slip_latch_ = true;
    }
  }
  else
  {
    // Only release latch if both velocity estimates agree that we are driving above a threshold
    if (vel_kf_ > slip_velocity_latch_release_ms_ && vel_wheels_ > slip_velocity_latch_release_ms_)
    {
      slip_latch_ = false;
    }
  }

  slip_msg_.slip_latch = slip_latch_;
  slip_msg_.slip = curr_raw_slip_;
  slip_msg_.slip_avg = slip_avg_;
  slip_msg_.vel_int = curr_vel_estimate_;
  slip_msg_.vel_avg = curr_vel_avg_;
  slip_msg_.vel_twst = vel_twst_;
  slip_msg_.vel_kf = vel_kf_;
  slip_msg_.slip = curr_slip_;
  slip_msg_.header.stamp = this->get_clock()->now();
  slip_pub_->publish(slip_msg_);
}

void SlipEstimateNode::actStateCallback(const cg_msgs::msg::ActuatorState::SharedPtr msg)
{
  // Get current magnitude of wheel velocity
  vel_wheels_ = std::abs(msg->wheel_velocity);
}

void SlipEstimateNode::slipTelemetryCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg) {

  // Calculate current raw slip value
  actual_front_vel_ = static_cast < float > (std::abs(msg->drive_vel_rear));
  actual_rear_vel_ = static_cast<float>(std::abs(msg->drive_vel_front)); // TODO: get value from config file as message constant

  // if (actual_rear_vel_ > nonzero_slip_thresh_) 
  // {
  //   // Calculate slip if it's above the calculation threshold
  //   // Expect 0 for no slip, 1 for 100% slip, clamp at zero
  //   curr_raw_slip_ = std::max(static_cast<float>(0.0), (actual_rear_vel_ - actual_front_vel_) / actual_rear_vel_);
  // }
  // else {
  //   curr_raw_slip_ = static_cast<float>(0.0);
  // }

  // // Filter slip estimate with moving average
  // updateMovingAverage(); // call to update curr_slip_avg_

  // // Publish the slip estimate
  // slip_msg_.slip = curr_slip_avg_;
  // slip_msg_.header.stamp = this->get_clock()->now();
  // slip_pub_->publish(slip_msg_);

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

// void SlipEstimateNode::updateVelMovingAverage()
// {

//   // Build up the filter
//   vel_window_.push_front(curr_vel_estimate_);
//   if (static_cast<int>(vel_window_.size()) > vel_filter_window_)
//   {
//     vel_window_.pop_back(); // remove oldest value
//   }

//   // Update the filter
//   curr_vel_avg_ = std::accumulate(vel_window_.begin(), vel_window_.end(), 0.0) / vel_window_.size();
// }

float SlipEstimateNode::updateMovingAverage(std::list<float> &list, float &new_val, int window_size)
{
  // Build up the filter
  list.push_front(new_val);
  if (static_cast<int>(list.size()) > window_size)
  {
    list.pop_back(); // remove oldest value
  }

  // Update the filter
  return std::accumulate(list.begin(), list.end(), 0.0) / list.size();
}

void SlipEstimateNode::globalCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Kalman filter velocity estimate
  Eigen::VectorXd z_(kf_m_);
  z_ << msg->pose.pose.position.x, msg->pose.pose.position.y;

  kf_vel_.predict();
  kf_vel_.update(z_);
  Eigen::VectorXd xhat_(kf_n_);
  xhat_ = kf_vel_.state(); // [x; xdot; y; ydot]
  vel_kf_ = sqrt(pow(xhat_(1), 2.0) + pow(xhat_(3), 2.0)); // / 0.00008298755187;



  // Initialize last time step
  if (global_init_)
  {
    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;
    last_t_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  }
  global_init_ = false;

  // Only sample every n steps
  // if (sampler_++ % sampling_steps_ != 0) {
  //   return;
  // }

  // Change in time
  delta_t_ = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - last_t_;

  // Make velocity estimate
  curr_vel_estimate_ = sqrt(
      pow(msg->pose.pose.position.x - last_x_, 2.0) +
      pow(msg->pose.pose.position.y - last_y_, 2.0)) / delta_t_;

  curr_vel_estimate_ = curr_vel_estimate_ / 0.00008298755187;

  vel_twst_ = sqrt(pow(msg->twist.twist.linear.x, 2.0) + pow(msg->twist.twist.linear.y, 2.0));

  vel_twst_ = vel_twst_ / 0.00008298755187;

  // Filter slip estimate with moving average
  curr_vel_avg_ = updateMovingAverage(vel_window_, curr_vel_estimate_, vel_filter_window_); // call to update filtered slip estimate

  // Update last time step
  last_x_ = msg->pose.pose.position.x;
  last_y_ = msg->pose.pose.position.y;
  last_t_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // if (actual_rear_vel_ > nonzero_slip_thresh_)
  // {
  //   // Calculate slip if it's above the calculation threshold
  //   // Expect 0 for no slip, 1 for 100% slip, clamp at zero
  //   curr_raw_slip_ = std::max(static_cast<float>(0.0), (actual_rear_vel_ - curr_vel_avg_) / actual_rear_vel_);
  // }
  // else
  // {
  //   curr_raw_slip_ = static_cast<float>(0.0);
  // }

  // // Evaluate slip latch conditions
  // if (!slip_latch_) {
  //   if (curr_raw_slip_ > slip_latch_thresh_) 
  //   {
  //     slip_latch_ = true;
  //   }
  // }
  // else {
  //   if (curr_vel_avg_ > slip_velocity_latch_release_)
  //   {
  //     slip_latch_ = false;
  //   }
  // }

  // Publish the slip estimate
  // slip_msg_.slip_latch = slip_latch_;
  // slip_msg_.vel_kf = vel_kf_;
  // slip_msg_.header.stamp = this->get_clock()->now();
  // slip_pub_->publish(slip_msg_);
}


// void SlipEstimateNode::encCallback(
//   const nav_msgs::msg::Odometry::SharedPtr enc_msg) {

//   last_enc_vel_ = sqrt(
//     pow(enc_msg->twist.twist.linear.x, 2.0) + 
//     pow(enc_msg->twist.twist.linear.y, 2.0));

//   enc_init = true;

// }

} // namespace odom
} // namespace cg
