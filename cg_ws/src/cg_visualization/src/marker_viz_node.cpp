#include "cg_visualization/marker_viz_node.hpp"

namespace cg {
namespace cg_visualization
{
  MarkerVizNode::MarkerVizNode() : Node("marker_viz_node") {

    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker_array", 1);

    // Timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MarkerVizNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "In constructor");
    crater1_.header.frame_id = "map";
    crater1_.header.stamp = this->get_clock()->now();
    crater1_.ns = "craters";
    crater1_.id = 0;
    crater1_.type = visualization_msgs::msg::Marker::CYLINDER;
    crater1_.action = visualization_msgs::msg::Marker::ADD;
    crater1_.pose.position.x = 1;
    crater1_.pose.position.y = 1;
    crater1_.pose.position.z = 1;
    crater1_.pose.orientation.x = 0.0;
    crater1_.pose.orientation.y = 0.0;
    crater1_.pose.orientation.z = 0.0;
    crater1_.pose.orientation.w = 1.0;
    crater1_.scale.x = 0.1;
    crater1_.scale.y = 0.1;
    crater1_.scale.z = 0.1;
    crater1_.color.a = 1.0; // Don't forget to set the alpha!
    crater1_.color.r = 0.0;
    crater1_.color.g = 1.0;
    crater1_.color.b = 0.0;
    markers_msg_.markers.push_back(crater1_);
  }

  void MarkerVizNode::timerCallback()
  {

    // Report the current mode
    RCLCPP_INFO(this->get_logger(), "In timer");
    markers_pub_->publish(markers_msg_);
  }

} // namespace cg_visualization
} // namespace cg