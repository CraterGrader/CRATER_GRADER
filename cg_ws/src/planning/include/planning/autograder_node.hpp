#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/slip.hpp>

namespace cg {
namespace planning {

class AutograderNode : public rclcpp::Node {

public:
  AutograderNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr tool_pos_pub_;
  rclcpp::Subscription<cg_msgs::msg::Slip>::SharedPtr slip_sub_;

  /* Message data */
  cg_msgs::msg::Slip slip_msg_;

  /* Callbacks */
  void slipCallback(const cg_msgs::msg::Slip::SharedPtr msg); // read slip estimate
  void timerCallback(); // publish tool control commands

  /* Variables */
  float curr_slip_estimate_;
  float slip_thresh_; // threshold for moving tool down to design or up to maximum position
  float half_deadband_; // don't move tool within range of slip_thresh_ +/- half_deadband_
  int design_blade_pos_; // blade height for grading design plane, target when slip is at thresh - half_deadband_
  int max_blade_pos_; // target when slip is at thresh + half_deadband_
  

  /* Parameters */  

}; // class AutograderNode

} // namespace planning
} // namespace cg
