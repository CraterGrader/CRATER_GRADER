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
  rclcpp::Subscription<cg_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_state_sub_;

  /* Callbacks */
  void trajectoryCallback(const cg_msgs::msg::Trajectory::SharedPtr msg);
  void robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /* Services */
  rclcpp::Service<cg_msgs::srv::UpdateTrajectory>::SharedPtr update_trajectory_server_;
  rclcpp::Service<cg_msgs::srv::EnableWorksystem>::SharedPtr enable_worksystem_server_;
  void updateTrajectory(cg_msgs::srv::UpdateTrajectory::Request::SharedPtr req, cg_msgs::srv::UpdateTrajectory::Response::SharedPtr res);
  void enableWorksystem(cg_msgs::srv::EnableWorksystem::Request::SharedPtr req, cg_msgs::srv::EnableWorksystem::Response::SharedPtr res);

  /* Controllers */
  LongitudinalController lon_controller_;
  LateralController lat_controller_;

  /* Parameters */  
  double longitudinal_velocity_kp_;
  double longitudinal_velocity_ki_;
  double longitudinal_velocity_kd_;

  double lateral_stanley_gain_;

  /* Variables */
  cg_msgs::msg::Trajectory current_trajectory_;
  bool worksystem_enabled_;

}; // class WorksystemControlNode

} // namespace motion_control
} // namespace cg
