#ifndef TELEOP__TELEOP_NODE_HPP
#define TELEOP__TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace cg {
namespace teleop {

class TeleopNode : public rclcpp::Node {

public:
  TeleopNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  /* Callbacks */
  // Callback for joystick input
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  int joy_axis_drive_i_ = 1; // Axis index for forward/backward drive
  int joy_axis_steer_i_ = 0; // Axis index for steer data
  double joy_axis_drive_state_ = 0.0;  // Forward/backward drive state (+1 max fwd, -1 max bwd)
  double joy_axis_steer_state_ = 0.0;  // Left/right steer state (+1 max left, -1 max right)
};


}  // namespace teleop
}  // namespace cg

#endif  // TELEOP__TELEOP_NODE_HPP