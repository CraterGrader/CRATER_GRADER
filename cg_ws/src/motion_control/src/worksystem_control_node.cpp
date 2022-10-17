#include "motion_control/worksystem_control_node.hpp"

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
  std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> global_callback_fn = std::bind(
      &WorksystemControlNode::robotStateCallback, this, std::placeholders::_1, global_robot_state_
  );
  std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> local_callback_fn = std::bind(
      &WorksystemControlNode::robotStateCallback, this, std::placeholders::_1, local_robot_state_
  );
  global_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered/ekf_global_node",1, global_callback_fn);
  local_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered/ekf_odom_node",1, local_callback_fn);

  // Initialize publishers
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
    "/autonomous_control_cmd", 1);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WorksystemControlNode::timerCallback, this)
  );

  // Load parameters
  this->declare_parameter<double>("longitudinal_velocity_kp", 1.0);
  this->get_parameter("longitudinal_velocity_kp", longitudinal_velocity_kp_);
  this->declare_parameter<double>("longitudinal_velocity_ki", 0.0);
  this->get_parameter("longitudinal_velocity_ki", longitudinal_velocity_ki_);
  this->declare_parameter<double>("longitudinal_velocity_kd", 0.0);
  this->get_parameter("longitudinal_velocity_kd", longitudinal_velocity_kd_);
  this->declare_parameter<double>("lateral_stanley_gain", 1.0);
  this->get_parameter("lateral_stanley_gain", lateral_stanley_gain_);
}

void WorksystemControlNode::timerCallback() {
  // TODO
}

void WorksystemControlNode::updateTrajectory(cg_msgs::srv::UpdateTrajectory::Request::SharedPtr req, cg_msgs::srv::UpdateTrajectory::Response::SharedPtr res)
{
  current_trajectory_ = req->trajectory; // Update current trajectory
  res->updated_trajectory = true; // Set response confirmation
}

void WorksystemControlNode::robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg, nav_msgs::msg::Odometry &out_msg) {
  out_msg = *msg;
}

void WorksystemControlNode::enableWorksystem(cg_msgs::srv::EnableWorksystem::Request::SharedPtr req, cg_msgs::srv::EnableWorksystem::Response::SharedPtr res)
{
  worksystem_enabled_ = req->enable_worksystem; // Enable/disable worksystem using service request
  res->worksystem_enabled = worksystem_enabled_; // Set response confirmation
}

}  // namespace motion_control
}  // namespace cg
