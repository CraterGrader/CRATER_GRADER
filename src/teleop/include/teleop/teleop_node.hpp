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
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) const;

  int joy_axis_fb_i_ = 1; // Axis index for forward/backward joystick data
  int joy_axis_lr_i_ = 0; // Axis index for left/right joystick data
  double joy_axis_fb_state_ = 0.0;  // Forward/backward joystick state (+1 max fwd, -1 max bwd)
  double joy_axis_lr_state_ = 0.0;  // Left/right joystick state (+1 max left, -1 max right)
};


}  // namespace teleop
}  // namespace cg

#endif  // TELEOP__TELEOP_NODE_HPP