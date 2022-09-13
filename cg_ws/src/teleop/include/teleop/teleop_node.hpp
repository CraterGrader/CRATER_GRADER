#ifndef TELEOP__TELEOP_NODE_HPP
#define TELEOP__TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cg_msgs/msg/mux_mode.hpp>
#include <cg_msgs/msg/actuator_command.hpp>

namespace cg {
namespace teleop {

class TeleopNode : public rclcpp::Node {

public:
  TeleopNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<cg_msgs::msg::MuxMode>::SharedPtr mode_sub_;            // Multiplexer mode number
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr act_cmd_sub_; // subscribe to the last actuator command so the relative teleop inputs start with existing state

  /* Callbacks */
  // Callback for joystick input
  void timerCallback();
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg);
  void actCmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);

  int joy_axis_drive_i_; // Axis index for forward/backward drive
  int joy_axis_steer_i_; // Axis index for steer data
  int joy_lower_tool_i_; // Axis index for tool lowering bumper data
  int joy_raise_tool_i_; // Axis index for tool raising bumper data

  int joy_tool_increment_; // Tool increment percent of full scale for each joystick bumper press

  double joy_axis_drive_state_ = 0.0;  // Forward/backward drive state (+1 max fwd, -1 max bwd)
  double joy_axis_steer_state_ = 0.0;  // Left/right steer state (+1 max left, -1 max right)
  double joy_tool_height_state_ = 0.0;  // Up/down tool state (+100 max height, -100 max height)

  bool joy_tool_pressed_ = false; //Records last pressed state to require independent presses

  uint8_t curr_mux_mode_; // Multiplexer node mode, used to know if command should be changed and to update starting command for additional teleop inputs
};


}  // namespace teleop
}  // namespace cg

#endif  // TELEOP__TELEOP_NODE_HPP
