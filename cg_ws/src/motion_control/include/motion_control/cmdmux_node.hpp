#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
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

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_sub_;
  rclcpp::Subscription<cg_msgs::msg::MuxMode>::SharedPtr mode_sub_TEST_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr autonomy_sub_;

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish in idle mode

  /* Message data */
  cg_msgs::msg::ActuatorCommand cmd_msg_;

  /* Callbacks */
  void modeCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void teleopCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  void autonomyCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  void timerCallback(); // For looping publish in idle mode

  /* Variables */
  uint8_t curr_mode_;
  uint8_t idle_mode_;
  uint8_t autograder_mode_;
  uint8_t full_autonomy_mode_;
  uint8_t full_teleop_mode_;
  cg_msgs::msg::ActuatorCommand last_cmd_;

  // Autograder boolean flags
  bool autograder_teleop_received_;
  bool autograder_autonomy_received_;

  /* Helpers */
  void asyncPublishAutograder();

}; // class CmdMuxNode

} // namespace cmdmux
} // namespace cg
