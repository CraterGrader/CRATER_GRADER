#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/mux_mode.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/slip.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace cg {
namespace planning {

class AutoGraderNode : public rclcpp::Node {

public:
  AutoGraderNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<cg_msgs::msg::Slip>::SharedPtr slip_sub_;
  rclcpp::Subscription<cg_msgs::msg::MuxMode>::SharedPtr mode_sub_; // Multiplexer mode number

  rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr telem_sub_;

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish rate

  /* Message data */
  cg_msgs::msg::ActuatorCommand cmd_msg_;

  /* Callbacks */
  void slipCallback(const cg_msgs::msg::Slip::SharedPtr msg); // read slip estimate
  void timerCallback(); // publish tool control commands
  void modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg);
  void telemCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);

  /* Variables */
  uint8_t curr_mux_mode_; // Multiplexer operating mode

  /* Parameters */  
  float slip_thresh_;
  float half_deadband_;
  double design_blade_pos_;
  double max_des_blade_pos_;

}; // class AutoGraderNode

} // namespace planning
} // namespace cg
