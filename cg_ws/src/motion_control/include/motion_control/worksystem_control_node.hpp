#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  void robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg, nav_msgs::msg::Odometry &out_msg);
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
}; // class WorksystemControlNode

} // namespace motion_control
} // namespace cg
