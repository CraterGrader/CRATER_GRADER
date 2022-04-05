#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/mux_mode.hpp>
#include <cg_msgs/msg/actuator_command.hpp>

// TODO: rename /teleop_cmd topic

namespace cg {
namespace cmdmux {

class CmdMuxNode : public rclcpp::Node {

public:
  CmdMuxNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;

  rclcpp::Subscription<cg_msgs::msg::MuxMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr autonomy_sub_;

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish in idle mode

  /* Message data */
  cg_msgs::msg::ActuatorCommand cmd_msg_;

  /* Callbacks */
  void modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg);
  void teleopCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  void autonomyCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  void timerCallback(); // For looping publish in idle mode

  /* Variables */
  uint8_t curr_mode_;

}; // class CmdMuxNode

} // namespace cmdmux
} // namespace cg
