#include "bearing/bearing_node.hpp"

namespace cg {
namespace bearing {

BearingNode::BearingNode() : Node("bearing_node") {
  // Initialize publishers and subscribers
  bearing_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/bearing", 1
  );
  tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 1, std::bind(&BearingNode::tfCallback, this, std::placeholders::_1)
  );

  // Load parameters
  this->declare_parameter<int>("box_width", 7.5);
  this->get_parameter("box_width", box_width_);
  this->declare_parameter<int>("box_length", 7.5);
  this->get_parameter("box_length", box_length_);
  this->declare_parameter<int>("box_height", 1);
  this->get_parameter("box_height", box_height_);
  this->declare_parameter<int>("mounting_height", 1);
  this->get_parameter("mounting_height", mounting_height_);
  this->declare_parameter<int>("tag_size", 0.3);
  this->get_parameter("tag_size", tag_size_);
}

void BearingNode::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  std_msgs::msg::Float64 bearing;
  tf2_msgs::msg::TFMessage tag_to_cam = tf2::Transform::inverse(msg->transforms[0].transform);
  //bearing = 
}

}  // namespace teleop
}  // namespace cg
