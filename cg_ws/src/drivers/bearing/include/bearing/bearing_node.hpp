#ifndef BEARING__BEARING_NODE_HPP
#define BEARING__BEARING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace cg {
namespace bearing {

class BearingNode : public rclcpp::Node {

public:
  BearingNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bearing_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

  /* Callbacks */
  // Callback for joystick input
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  double box_length_; // Sandbox length perpendicular to GHC roll-up gates
  double box_width_; // Sandbox width parallel to GHC roll-up gates
  double box_height_; // Sandbox height from ground plane

  double mounting_height_; // Height of top edge from the ground plane
  double tag_size_; // Size of apriltag's inner box
};

}  // namespace bearing
}  // namespace cg

#endif  // BEARING_BEARING_NODE_HPP
