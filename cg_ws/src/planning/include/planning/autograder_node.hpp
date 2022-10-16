#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/mux_mode.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>

namespace cg {
namespace planning {

class AutoGraderNode : public rclcpp::Node {

public:
  AutoGraderNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<cg_msgs::msg::MuxMode>::SharedPtr mode_sub_; // Multiplexer mode number

  rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr telem_sub_;

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish rate

  /* Message data */
  cg_msgs::msg::ActuatorCommand cmd_msg_;

  /* Callbacks */
  void timerCallback(); // publish tool control commands
  void modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg);
  void telemCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);

  /* Variables */
  uint8_t curr_mux_mode_; // Multiplexer operating mode
  bool driving_forward_; 

  /* Parameters */
  double design_blade_pos_;
  double raised_blade_pos_;

}; // class AutoGraderNode

} // namespace planning
} // namespace cg
