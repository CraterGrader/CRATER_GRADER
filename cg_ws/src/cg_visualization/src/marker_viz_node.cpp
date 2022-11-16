#include "cg_visualization/marker_viz_node.hpp"

namespace cg {
namespace cg_visualization
{
  MarkerVizNode::MarkerVizNode() : Node("marker_viz_node") {
    // Tool pose
    tool_pose_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/viz/planning/tool_pose_viz", 1);
    enc_tele_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
        "/encoder_telemetry", 1, std::bind(&MarkerVizNode::updateToolViz, this, std::placeholders::_1));
    // Steer angle
    arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 1);
    act_state_sub_ = this->create_subscription<cg_msgs::msg::ActuatorState>(
        "/actuator/state", 1, std::bind(&MarkerVizNode::actStateCallback, this, std::placeholders::_1));
    
    // Timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MarkerVizNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "In constructor");

    // Arrow parameters
    this->declare_parameter<float>("arrow_scale_x", 0.5);
    this->get_parameter("arrow_scale_x", arrow_scale_x_);
    this->declare_parameter<float>("arrow_scale_y", 0.05);
    this->get_parameter("arrow_scale_y", arrow_scale_y_);
    this->declare_parameter<float>("arrow_scale_z", 0.05);
    this->get_parameter("arrow_scale_z", arrow_scale_z_);
    this->declare_parameter<float>("arrow_r", 1.0);
    this->get_parameter("arrow_r", arrow_r_);
    this->declare_parameter<float>("arrow_g", 0.0);
    this->get_parameter("arrow_g", arrow_g_);
    this->declare_parameter<float>("arrow_b", 1.0);
    this->get_parameter("arrow_b", arrow_b_);

    RCLCPP_INFO(this->get_logger(), "In Tool / Steer Viz Constructor");

    toolPoseVizInit();
  }

  void MarkerVizNode::timerCallback()
  {
    // Publish both craters and the steering angle
    arrow_pub_->publish(steer_angle_arrow_);
    tool_array.markers.clear();
    tool_array.markers.push_back(tool1);
    tool_array.markers.push_back(tool2);
    tool_array.markers.push_back(tool_text);
    tool_array.markers.push_back(body);
    tool_array.markers.push_back(ground);
    tool_pose_pub_->publish(tool_array);
  }

  void MarkerVizNode::updateToolViz(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "In update tool viz");
    float msg_tool_pose = static_cast<float>(msg->tool_pos)/328.0;
    RCLCPP_INFO(this->get_logger(), std::to_string(msg_tool_pose).c_str());
    tool_pose_ = msg_tool_pose;

    // Update length and position of tool
    tool1.pose.position.y = 2.25 - 0.25 * tool_pose_/80.0;
    tool1.scale.y = 0.5 + 0.3 * tool_pose_/80.0;
    tool2.pose.position.y = tool1.pose.position.y - tool1.scale.y/2;
  }

  void MarkerVizNode::toolPoseVizInit()
  {
    RCLCPP_INFO(this->get_logger(), "Tool pose viz initialization");
    // Tool Text Description
    tool_text.header.frame_id = "map";
    tool_text.header.stamp = this->get_clock()->now();
    tool_text.ns = "tool_text";
    tool_text.id = 1;
    tool_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    tool_text.action = visualization_msgs::msg::Marker::ADD;
    tool_text.pose.position.x = -2.0;
    tool_text.pose.position.y = 4.0;
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
    tool_text.text = "Tool_Position";
    // Vehicle Body
    body.header.frame_id = "map";
    body.header.stamp = this->get_clock()->now();
    body.ns = "body";
    body.id = 2;
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
    ground.id = 3;
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
    tool1.id = 4;
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
    tool2.id = 5;
    tool2.type = visualization_msgs::msg::Marker::CUBE;
    tool2.action = visualization_msgs::msg::Marker::ADD;
    tool2.pose.position.x = -2.2;
    tool2.pose.position.y = 1.5;
    tool2.pose.position.z = 0.1;
    tool2.pose.orientation = tool_text.pose.orientation;
    tool2.scale.x = 0.2;
    tool2.scale.y = 0.4;
    tool2.scale.z = 0.1;
    tool2.color = tool1.color;

    tool_array.markers.clear();
    tool_array.markers.push_back(tool1);
    tool_array.markers.push_back(tool2);
    tool_array.markers.push_back(tool_text);
    tool_array.markers.push_back(body);
    tool_array.markers.push_back(ground);
    tool_pose_pub_->publish(tool_array);
  }


  void MarkerVizNode::actStateCallback(const cg_msgs::msg::ActuatorState::SharedPtr msg)
  {
    // Create arrow
    steer_angle_arrow_.header.frame_id = "base_link";
    steer_angle_arrow_.header.stamp = this->get_clock()->now();
    steer_angle_arrow_.ns = "kinematics";
    steer_angle_arrow_.id = 0;
    steer_angle_arrow_.type = visualization_msgs::msg::Marker::ARROW;
    steer_angle_arrow_.action = visualization_msgs::msg::Marker::ADD;
    steer_angle_arrow_.pose.position.x = 0.2;
    steer_angle_arrow_.pose.position.y = 0;
    steer_angle_arrow_.pose.position.z = 0;

    // rpy
    tf2::Quaternion q;
    q.setRPY(0, 0, -msg->steer_position); // Not sure why the steer angle needs to be negative
    steer_angle_arrow_.pose.orientation.x = q.x();
    steer_angle_arrow_.pose.orientation.y = q.y();
    steer_angle_arrow_.pose.orientation.z = q.z();
    steer_angle_arrow_.pose.orientation.w = q.w();

    steer_angle_arrow_.scale.x = arrow_scale_x_;
    steer_angle_arrow_.scale.y = arrow_scale_y_;
    steer_angle_arrow_.scale.z = arrow_scale_z_;
    steer_angle_arrow_.color.a = 1.0; // Don't forget to set the alpha!
    steer_angle_arrow_.color.r = arrow_r_;
    steer_angle_arrow_.color.g = arrow_g_;
    steer_angle_arrow_.color.b = arrow_b_;
  }
} // namespace cg_visualization
} // namespace cg