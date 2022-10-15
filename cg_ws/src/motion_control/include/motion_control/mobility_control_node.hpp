#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "motion_control/lateral_controller.hpp"
#include "motion_control/longitudinal_controller.hpp"

namespace cg {
namespace motion_control {

class MobilityControlNode : public rclcpp::Node {

public:
  MobilityControlNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<cg_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_state_sub_;

  /* Callbacks */
  void trajectoryCallback(const cg_msgs::msg::Trajectory::SharedPtr msg);
  void robotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /* Controllers */
  LongitudinalController lon_controller_;
  LateralController lat_controller_;

  /* Parameters */  
  double longitudinal_velocity_kp_;
  double longitudinal_velocity_ki_;
  double longitudinal_velocity_kd_;

  double lateral_stanley_gain_;

}; // class MobilityControlNode

} // namespace motion_control
} // namespace cg
