#include "cg_visualization/marker_viz_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace cg {
namespace cg_visualization
{
  MarkerVizNode::MarkerVizNode() : Node("marker_viz_node") {

    tool_pose_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/viz/planning/tool_pose", 1);
    enc_tele_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
        "/encoder_telemetry", 1, std::bind(&MarkerVizNode::updateToolViz, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "In constructor");

    toolPoseVizInit();
  }

  void MarkerVizNode::updateToolViz(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg)
  {
    float tool_pose = static_cast<float>(msg->tool_pos);

    // Update length and position of tool
    tool1.pose.position.y = 2.5 - 0.5 * tool_pose/70.0;
    tool1.scale.y = tool_pose/70.0;
    tool2.pose.position.y = tool1.pose.position.y - tool1.scale.y/2;
    tool_pose_pub_->publish(tool1);
    tool_pose_pub_->publish(tool2);
  }

  void MarkerVizNode::toolPoseVizInit()
  {
    // RCLCPP_INFO(this->get_logger(), "Starting Tool Pose Viz");
    // Text Description
    viz_text.header.frame_id = "map";
    viz_text.header.stamp = this->get_clock()->now();
    viz_text.ns = "viz_text";
    viz_text.id = 0;
    viz_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    viz_text.action = visualization_msgs::msg::Marker::ADD;
    viz_text.pose.position.x = -2;
    viz_text.pose.position.y = 3.5;
    viz_text.pose.position.z = 0;
    viz_text.pose.orientation.x = 0.0;
    viz_text.pose.orientation.y = 0.0;
    viz_text.pose.orientation.z = 0.0;
    viz_text.pose.orientation.w = 1.0;
    viz_text.scale.x = 0.1;
    viz_text.scale.y = 0.1;
    viz_text.scale.z = 0.4;
    viz_text.color.a = 1.0;
    viz_text.color.r = 0.0;
    viz_text.color.g = 0.0;
    viz_text.color.b = 0.0;
    viz_text.text = "Tool Position";
    // Vehicle Body
    body.header.frame_id = "map";
    body.header.stamp = this->get_clock()->now();
    body.ns = "body";
    body.id = 1;
    body.type = visualization_msgs::msg::Marker::CUBE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = -2.0;
    body.pose.position.y = 3.0;
    body.pose.position.z = 0.0;
    body.pose.orientation = viz_text.pose.orientation;
    body.scale.x = 3;
    body.scale.y = 1;
    body.scale.z = 0.1;
    body.color.a = 1.0;
    body.color.r = 0.6;
    body.color.g = 0.6;
    body.color.b = 0.6;
    // Ground (Sand)
    ground.header.frame_id = "map";
    ground.header.stamp = this->get_clock()->now();
    ground.ns = "ground";
    ground.id = 2;
    ground.type = visualization_msgs::msg::Marker::CUBE;
    ground.action = visualization_msgs::msg::Marker::ADD;
    ground.pose.position.x = -2.0;
    ground.pose.position.y = 0.75;
    ground.pose.position.z = 0;
    ground.pose.orientation = viz_text.pose.orientation;
    ground.scale.x = 3.0;
    ground.scale.y = 1.5;
    ground.scale.z = 0.1;
    ground.color.a = 1.0;
    ground.color.r = 0.961;
    ground.color.g = 0.961;
    ground.color.b = 0.863;
    // Tool Blocks
    tool1.header.frame_id = "map";
    tool1.header.stamp = this->get_clock()->now();
    tool1.ns = "tool1";
    tool1.id = 3;
    tool1.type = visualization_msgs::msg::Marker::CUBE;
    tool1.action = visualization_msgs::msg::Marker::ADD;
    tool1.pose.position.x = -2;
    tool1.pose.position.y = 2.0;
    tool1.pose.position.z = 0.1;
    tool1.pose.orientation = viz_text.pose.orientation;
    tool1.scale.x = 0.2;
    tool1.scale.y = 1.0;
    tool1.scale.z = 0.1;
    tool1.color.a = 1.0;
    tool1.color.r = 1.0;
    tool1.color.g = 0.0;
    tool1.color.b = 0.0;
    tool2.header.frame_id = "map";
    tool2.header.stamp = this->get_clock()->now();
    tool2.ns = "tool2";
    tool2.id = 4;
    tool2.type = visualization_msgs::msg::Marker::CUBE;
    tool2.action = visualization_msgs::msg::Marker::ADD;
    tool2.pose.position.x = -2.2;
    tool2.pose.position.y = 1.5;
    tool2.pose.position.z = 0.1;
    tool2.pose.orientation = viz_text.pose.orientation;
    tool2.scale.x = 0.2;
    tool2.scale.y = 0.5;
    tool2.scale.z = 0.1;
    tool2.color = tool1.color;

    tool_pose_pub_->publish(viz_text);
    tool_pose_pub_->publish(body);
    tool_pose_pub_->publish(ground);
    tool_pose_pub_->publish(tool1);
    tool_pose_pub_->publish(tool2);
  }

} // namespace cg_visualization
} // namespace cg