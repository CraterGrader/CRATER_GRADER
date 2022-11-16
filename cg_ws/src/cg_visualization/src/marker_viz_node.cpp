#include "cg_visualization/marker_viz_node.hpp"

namespace cg {
namespace cg_visualization
{
  MarkerVizNode::MarkerVizNode() : Node("marker_viz_node") {

    tool_pose_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/viz/planning/tool_pose_viz", 1);
    steer_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/viz/planning/steer_pose_viz", 1);
    enc_tele_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
        "/encoder_telemetry", 1, std::bind(&MarkerVizNode::updateToolViz, this, std::placeholders::_1));
    act_cmd_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
        "/actuator_cmd", 1, std::bind(&MarkerVizNode::updateSteerViz, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered/ekf_global_node", 1, std::bind(&MarkerVizNode::poseUpdateViz,
        this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "In Tool / Steer Viz Constructor");

    toolPoseVizInit();
  }

  void MarkerVizNode::updateToolViz(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg)
  {
    float msg_tool_pose = static_cast<float>(msg->tool_pos)/129.0;
    float diff = std::fabs(tool_pose_ - msg_tool_pose);
    float scaling = std::fabs(tool_pose_ + msg_tool_pose);
    if (diff < std::numeric_limits<float>::epsilon() * scaling) {
      tool_pose_ = msg_tool_pose;

      // Update length and position of tool
      tool1.pose.position.y = 2.5 - 0.5 * tool_pose_/80.0;
      tool1.scale.y = tool_pose_/80.0;
      tool2.pose.position.y = tool1.pose.position.y - tool1.scale.y/2;
      tool_pose_pub_->publish(tool1);
      tool_pose_pub_->publish(tool2);
    }
  }

  void MarkerVizNode::updateSteerViz(const cg_msgs::msg::ActuatorCommand::SharedPtr msg)
  {
    steer_pose_ = msg->steer_position / 100.0 * 16.0 / 180.0 * M_PI;
  }

  void MarkerVizNode::poseUpdateViz(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Set header and frame name
    steer.header.stamp = this->get_clock()->now();
    steer.header.frame_id = "base_link";

    // Set the Pose relative to the robot frame to zeros
    steer.pose.position.x = 0.0;
    steer.pose.position.y = 0.0;
    steer.pose.position.z = 0.0;

    // Combine the current yaw and steer pose
    tf2::Quaternion q_yaw, q_steer, q_new;
    q_steer.setRPY(0.0, 0.0, steer_pose_);
    tf2::convert(msg->pose.pose.orientation, q_yaw);
    q_new = q_steer * q_yaw;
    q_new.normalize();

    // Set the quarternion in the message
    steer.pose.orientation.x = q_new.x();
    steer.pose.orientation.y = q_new.y();
    steer.pose.orientation.z = q_new.z();
    steer.pose.orientation.w = q_new.w();

    // Publish the steering position
    steer_pose_pub_->publish(steer);
  }

  void MarkerVizNode::toolPoseVizInit()
  {
    // RCLCPP_INFO(this->get_logger(), "Starting Tool Pose Viz");
    // Tool Text Description
    tool_text.header.frame_id = "map";
    tool_text.header.stamp = this->get_clock()->now();
    tool_text.ns = "tool_text";
    tool_text.id = 0;
    tool_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    tool_text.action = visualization_msgs::msg::Marker::ADD;
    tool_text.pose.position.x = -2.0;
    tool_text.pose.position.y = 3.5;
    tool_text.pose.position.z = 0;
    tool_text.pose.orientation.x = 0.0;
    tool_text.pose.orientation.y = 0.0;
    tool_text.pose.orientation.z = 0.0;
    tool_text.pose.orientation.w = 1.0;
    tool_text.scale.x = 0.1;
    tool_text.scale.y = 0.1;
    tool_text.scale.z = 0.4;
    tool_text.color.a = 1.0;
    tool_text.color.r = 0.0;
    tool_text.color.g = 0.0;
    tool_text.color.b = 0.0;
    tool_text.text = "Tool Position";
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
    body.pose.orientation = tool_text.pose.orientation;
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
    ground.pose.orientation = tool_text.pose.orientation;
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
    tool1.pose.position.x = -2.0;
    tool1.pose.position.y = 2.0;
    tool1.pose.position.z = 0.1;
    tool1.pose.orientation = tool_text.pose.orientation;
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
    tool2.pose.orientation = tool_text.pose.orientation;
    tool2.scale.x = 0.2;
    tool2.scale.y = 0.5;
    tool2.scale.z = 0.1;
    tool2.color = tool1.color;

    tool_pose_pub_->publish(tool_text);
    tool_pose_pub_->publish(body);
    tool_pose_pub_->publish(ground);
    tool_pose_pub_->publish(tool1);
    tool_pose_pub_->publish(tool2);
  }

} // namespace cg_visualization
} // namespace cg