#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/trajectory.hpp>
#include "motion_control/pid_controller.hpp"

namespace cg {
namespace motion_control {

class MotionControlNode : public rclcpp::Node {

public:
  MotionControlNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<cg_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

  /* Callbacks */
  void trajectoryCallback(const cg_msgs::msg::Trajectory::SharedPtr msg);

  /* Controllers */
  PIDController longitudinal_controller_;

  /* Parameters */  
  double longitudinal_velocity_kp_;
  double longitudinal_velocity_ki_;
  double longitudinal_velocity_kd_;

  double lateral_stanley_gain_;

}; // class MotionControlNode

} // namespace motion_control
} // namespace cg
