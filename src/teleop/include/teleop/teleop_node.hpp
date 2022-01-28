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
};


}  // namespace teleop
}  // namespace cg

#endif  // TELEOP__TELEOP_NODE_HPP