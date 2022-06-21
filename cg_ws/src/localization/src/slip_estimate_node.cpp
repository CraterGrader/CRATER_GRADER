#include "localization/slip_estimate_node.hpp"

namespace cg {
namespace slip {

SlipEstimateNode::SlipEstimateNode() : Node("slip_estimate_node") {
  
  // Initialize publishers and subscribers
  slip_pub_ = this->create_publisher<cg_msgs::msg::Slip>(
    "/slip_estimate", 1
  );

  uwb_avg_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb_beacon/average_tag", 1, std::bind(&SlipEstimateNode::uwbAvgCallback, this, std::placeholders::_1));

  act_sub_ = this->create_subscription<cg_msgs::msg::ActuatorState>(
      "/actuator/state", 1, std::bind(&SlipEstimateNode::actStateCallback, this, std::placeholders::_1));

  // Timer callback
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SlipEstimateNode::timerCallback, this));

  // Load parameters
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
  this->declare_parameter<double>("kf_dt", 0.05);
  this->get_parameter("kf_dt", kf_dt_);

  this->declare_parameter<int>("vel_kf_window_size", 10);
  this->get_parameter("vel_kf_window_size", vel_kf_window_size_);

  this->declare_parameter<float>("nonzero_slip_thresh_wheel_ms", 0.1);
  this->get_parameter("nonzero_slip_thresh_wheel_ms", nonzero_slip_thresh_wheel_ms_);
  this->declare_parameter<float>("nonzero_slip_thresh_vehicle_ms_", 0.07);
  this->get_parameter("nonzero_slip_thresh_vehicle_ms_", nonzero_slip_thresh_vehicle_ms_);

  this->declare_parameter<float>("slip_latch_thresh_ms", 0.8);
  this->get_parameter("slip_latch_thresh_ms", slip_latch_thresh_ms_);
  this->declare_parameter<float>("slip_velocity_latch_release_ms", 0.06);
  this->get_parameter("slip_velocity_latch_release_ms", slip_velocity_latch_release_ms_);

  this->declare_parameter<int>("slip_window_size", 10);
  this->get_parameter("slip_window_size", slip_window_size_);

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

  // Construct the filter using initializer list with temp object
  cg::localization::KalmanFilterLinear kf_vel_tmp_(A_, H_, Q_, R_, P_);
  kf_vel_ = kf_vel_tmp_; // Assign temp object to the actual object that is declared in node header

  // Best guess of initial states
  x0_ << 0, 0, 0, 0; // [x; xdot; y; ydot]
  kf_vel_.init(x0_);
}

void SlipEstimateNode::timerCallback()
{
  // Calculate slip estimate if wheels are moving fast enough, and vehicle is moving too slow
  if (vel_wheels_ > nonzero_slip_thresh_wheel_ms_ && vel_kf_avg_ < nonzero_slip_thresh_vehicle_ms_)
  {
    // Expect 0 for no slip, 1 for 100% slip, clamp at zero so no negative slip (only using magnitudes)
    curr_slip_ = std::max(static_cast<float>(0.0), (vel_wheels_ - vel_kf_avg_) / vel_wheels_);
  }
  else
  {
    curr_slip_ = static_cast<float>(0.0);
  }

  // Use moving average to smooth slip estimate
  slip_avg_ = updateMovingAverage(slip_window_, curr_slip_, slip_window_size_);

  // Evaluate slip latch conditions
  if (!slip_latch_)
  {
    // Activate latch if average slip is above a threshold, and vehicle speed is below a threshold
    if (slip_avg_ > slip_latch_thresh_ms_)
    {
      slip_latch_ = true;
    }
  }
  else
  {
    // Only release latch if both velocity estimates agree that we are driving above a threshold
    if (vel_kf_avg_ > slip_velocity_latch_release_ms_ && vel_wheels_ > slip_velocity_latch_release_ms_)
    {
      slip_latch_ = false;
    }
  }

  // Publish the message; some extra values included to assist testing
  slip_msg_.slip = curr_slip_;
  slip_msg_.slip_avg = slip_avg_;
  slip_msg_.slip_latch = slip_latch_;
  slip_msg_.vel = vel_kf_;
  slip_msg_.vel_avg = vel_kf_avg_;
  slip_msg_.header.stamp = this->get_clock()->now();
  slip_pub_->publish(slip_msg_);
}

void SlipEstimateNode::actStateCallback(const cg_msgs::msg::ActuatorState::SharedPtr msg)
{
  // Get current magnitude of wheel velocity
  vel_wheels_ = std::abs(msg->wheel_velocity);
}

void SlipEstimateNode::uwbAvgCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Kalman filter velocity estimate
  Eigen::VectorXd z_(kf_m_);
  z_ << msg->pose.pose.position.x, msg->pose.pose.position.y;

  // Cycle estimator
  kf_vel_.predict();
  kf_vel_.update(z_);

  // Extract estimate of velocity magnitude
  Eigen::VectorXd xhat_(kf_n_);
  xhat_ = kf_vel_.state(); // [x; xdot; y; ydot]
  vel_kf_ = sqrt(pow(xhat_(1), 2.0) + pow(xhat_(3), 2.0));

  // Update moving average
  vel_kf_avg_ = updateMovingAverage(vel_kf_window_, vel_kf_, vel_kf_window_size_);
}

float SlipEstimateNode::updateMovingAverage(std::list<float> &list, const float &new_val, const int &window_size)
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

} // namespace odom
} // namespace cg
