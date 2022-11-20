#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
#include <std_msgs/msg/float64.hpp>
#include "motion_control/lateral_controller.hpp"
#include "motion_control/longitudinal_controller.hpp"
#include <cg_msgs/srv/update_trajectory.hpp> // Service for updating current trajectory
#include <cg_msgs/srv/enable_worksystem.hpp> // Service to enable/disable worksystem controller
namespace cg {
namespace motion_control {

class WorksystemControlNode : public rclcpp::Node {

public:
  WorksystemControlNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_robot_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_robot_state_sub_;
  rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr encoder_telemetry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Debugging publishers */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lat_debug_cross_track_err_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lat_debug_heading_err_pub_;

  /* Callbacks */
  void globalRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void localRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void encoderTelemetryCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);
  void timerCallback();

  /* Services */
  rclcpp::Service<cg_msgs::srv::UpdateTrajectory>::SharedPtr update_trajectory_server_;
  rclcpp::Service<cg_msgs::srv::EnableWorksystem>::SharedPtr enable_worksystem_server_;
  void updateTrajectory(cg_msgs::srv::UpdateTrajectory::Request::SharedPtr req, cg_msgs::srv::UpdateTrajectory::Response::SharedPtr res);
  void enableWorksystem(cg_msgs::srv::EnableWorksystem::Request::SharedPtr req, cg_msgs::srv::EnableWorksystem::Response::SharedPtr res);

  /* Controllers */
  std::unique_ptr<LongitudinalController> lon_controller_;
  std::unique_ptr<LateralController> lat_controller_;

  /* Parameters */  
  PIDParams pid_params_;
  double lateral_stanley_gain_;
  double lateral_stanley_softening_constant_;

  /* Variables */
  cg_msgs::msg::Trajectory current_trajectory_;
  bool worksystem_enabled_ = false;
  nav_msgs::msg::Odometry global_robot_state_;
  nav_msgs::msg::Odometry local_robot_state_;
  cg_msgs::msg::ActuatorCommand cmd_msg_;
  std::list<float> cmg_msg_steer_window_;
  std::list<float> cmg_msg_drive_window_;
  size_t cmd_msg_filter_window_size_;
  int traj_idx_ = 0; // used for tracking what index on trajectory is closest to current pose, monotonically increasing, reset when new traj given

  float steer_speed_ = 0.0f;
  float last_steer_pos_front_ = 0.0f;
  float last_steer_pos_rear_ = 0.0f;
  double last_wheel_velocity_ = 0.0;
  double max_wheel_velocity_delta_ = 0.0;
  double tlast_;
  double delta_t_; 

  std::list<float> steer_velocity_window_;
  size_t steer_speed_filter_window_size_;

  /* Helpers */
  // Update list of values for moving average, modify the list being passed as an argument
  float updateMovingAverage(std::list<float> &list, const float &new_val, const size_t &window_size);

}; // class WorksystemControlNode

} // namespace motion_control
} // namespace cg
