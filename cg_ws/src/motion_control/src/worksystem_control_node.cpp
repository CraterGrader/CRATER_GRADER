#include <limits>
#include "motion_control/worksystem_control_node.hpp"
#include <iostream> // DEBUG
namespace cg {
namespace motion_control {

WorksystemControlNode::WorksystemControlNode() : Node("worksystem_control_node") {

  // Initialize services
  update_trajectory_server_ = this->create_service<cg_msgs::srv::UpdateTrajectory>(
      "update_trajectory_server",
      std::bind(&WorksystemControlNode::updateTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  enable_worksystem_server_ = this->create_service<cg_msgs::srv::EnableWorksystem>(
      "enable_worksystem_server",
      std::bind(&WorksystemControlNode::enableWorksystem, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize subscribers
  // Casting to std::function is necessary when defining callbacks with additional parameters
  // due to a bug in ROS2 (More info: https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/)
  // std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> global_callback_fn = std::bind(
  //     &WorksystemControlNode::robotStateCallback, this, std::placeholders::_1, global_robot_state_
  // );
  // std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> local_callback_fn = std::bind(
  //     &WorksystemControlNode::robotStateCallback, this, std::placeholders::_1, local_robot_state_
  // );
  // global_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/odometry/filtered/ekf_global_node",1, global_callback_fn);
  // local_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/odometry/filtered/ekf_odom_node",1, local_callback_fn);

  global_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered/ekf_global_node", 1, std::bind(&WorksystemControlNode::globalRobotStateCallback, this, std::placeholders::_1));

  local_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered/ekf_odom_node", 1, std::bind(&WorksystemControlNode::localRobotStateCallback, this, std::placeholders::_1));

  // Initialize publishers
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
      "/autonomy_cmd", 1);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WorksystemControlNode::timerCallback, this)
  );

  // Debug publishers
  lat_debug_cross_track_err_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/lat_control_debug/cross_track_error", 1);
  lat_debug_heading_err_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/lat_control_debug/heading_error", 1);

  // Load parameters
  this->declare_parameter<double>("longitudinal_velocity_kp", 1.0);
  this->get_parameter("longitudinal_velocity_kp", pid_params_.kp);
  this->declare_parameter<double>("longitudinal_velocity_ki", 0.0);
  this->get_parameter("longitudinal_velocity_ki", pid_params_.ki);
  this->declare_parameter<double>("longitudinal_velocity_kd", 0.0);
  this->get_parameter("longitudinal_velocity_kd", pid_params_.kd);
  this->declare_parameter<double>("longitudinal_velocity_dt", 0.01);
  this->get_parameter("longitudinal_velocity_dt", pid_params_.dt);
  this->declare_parameter<double>("longitudinal_velocity_integral_sat_min", -std::numeric_limits<double>::infinity());
  this->get_parameter("longitudinal_velocity_integral_sat_min", pid_params_.integral_sat_min);
  this->declare_parameter<double>("longitudinal_velocity_integral_sat_max", std::numeric_limits<double>::infinity());
  this->get_parameter("longitudinal_velocity_integral_sat_max", pid_params_.integral_sat_max);
  this->declare_parameter<double>("longitudinal_velocity_output_sat_min", -std::numeric_limits<double>::infinity());
  this->get_parameter("longitudinal_velocity_output_sat_min", pid_params_.output_sat_min);
  this->declare_parameter<double>("longitudinal_velocity_output_sat_max", std::numeric_limits<double>::infinity());
  this->get_parameter("longitudinal_velocity_output_sat_max", pid_params_.output_sat_max);
  this->declare_parameter<double>("lateral_stanley_gain", 1.0);
  this->get_parameter("lateral_stanley_gain", lateral_stanley_gain_);
  this->declare_parameter<double>("lateral_stanley_softening_constant", 1.0);
  this->get_parameter("lateral_stanley_softening_constant", lateral_stanley_softening_constant_);

  // Initialize controllers
  lon_controller_ = std::make_unique<LongitudinalController>(LongitudinalController(pid_params_));
  lat_controller_ = std::make_unique<LateralController>(LateralController(lateral_stanley_gain_, lateral_stanley_softening_constant_));
}

void WorksystemControlNode::timerCallback() {
  // Don't publish anything if the worksystem is not enabled
  if (!worksystem_enabled_) {
    return;
  }

  // TODO implement gain scheduling if deemed necessary
  lon_controller_->setGains(pid_params_.kp, pid_params_.ki, pid_params_.kd);

  // Use global position from map frame and smooth velocity from odom frame as current state
  nav_msgs::msg::Odometry current_state = global_robot_state_;
  current_state.child_frame_id = local_robot_state_.child_frame_id;
  current_state.twist = local_robot_state_.twist;

  // Compute control command
  cg_msgs::msg::ActuatorCommand cmd_msg;
  cmd_msg.header.stamp = this->get_clock()->now();
  cmd_msg.wheel_velocity = lon_controller_->computeDrive(current_trajectory_, current_state);
  cmd_msg.steer_position = lat_controller_->computeSteer(current_trajectory_, current_state);
  // TODO compute cmd.tool_position once ToolController is available

  // Clamp these commands just in case cmd_mux doesn't handle acutator limits
  cmd_msg.wheel_velocity = std::max(-100.0, std::min(cmd_msg.wheel_velocity, 100.0)); // [-100.0, 100.0]
  cmd_msg.steer_position = std::max(-100.0, std::min(cmd_msg.steer_position, 100.0)); // [-100.0, 100.0]
  cmd_msg.tool_position = std::max(0.0, std::min(cmd_msg.tool_position, 100.0));       // [0.0, 100.0]

  // Publish control command
  cmd_pub_->publish(cmd_msg);

  // Publish debug variables
  auto debug = lat_controller_->getDebug();
  std_msgs::msg::Float64 debug_msg;
  debug_msg.data = debug.cross_track_err;
  lat_debug_cross_track_err_pub_->publish(debug_msg);
  debug_msg.data = debug.heading_err;
  lat_debug_heading_err_pub_->publish(debug_msg);
}

void WorksystemControlNode::updateTrajectory(cg_msgs::srv::UpdateTrajectory::Request::SharedPtr req, cg_msgs::srv::UpdateTrajectory::Response::SharedPtr res)
{
  current_trajectory_ = req->trajectory; // Update current trajectory
  res->updated_trajectory = true; // Set response confirmation
}

// void WorksystemControlNode::robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg, nav_msgs::msg::Odometry &out_msg) {
//   out_msg = *msg;
// }

void WorksystemControlNode::localRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  local_robot_state_ = *msg;
}

void WorksystemControlNode::globalRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  global_robot_state_ = *msg;
}

void WorksystemControlNode::enableWorksystem(cg_msgs::srv::EnableWorksystem::Request::SharedPtr req, cg_msgs::srv::EnableWorksystem::Response::SharedPtr res)
{
  worksystem_enabled_ = req->enable_worksystem; // Enable/disable worksystem using service request
  res->worksystem_enabled = worksystem_enabled_; // Set response confirmation
}

}  // namespace motion_control
}  // namespace cg
