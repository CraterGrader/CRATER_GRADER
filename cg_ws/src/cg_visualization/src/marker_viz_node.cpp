#include "cg_visualization/marker_viz_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace cg {
namespace cg_visualization
{
  MarkerVizNode::MarkerVizNode() : Node("marker_viz_node") {

    craters_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization_marker_array", 1);
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

    // Crater parameters
    this->declare_parameter<float>("crater1x", 0.0);
    this->get_parameter("crater1x", crater1x_);
    this->declare_parameter<float>("crater1y", 0.0);
    this->get_parameter("crater1y", crater1y_);
    this->declare_parameter<int>("crater1id", 0);
    this->get_parameter("crater1id", crater1id_);
    this->declare_parameter<float>("crater2x", 0.0);
    this->get_parameter("crater2x", crater2x_);
    this->declare_parameter<float>("crater2y", 0.0);
    this->get_parameter("crater2y", crater2y_);
    this->declare_parameter<int>("crater2id", 1);
    this->get_parameter("crater2id", crater2id_);

    this->declare_parameter<float>("crater_scale_x", 0.1);
    this->get_parameter("crater_scale_x", crater_scale_x_);
    this->declare_parameter<float>("crater_scale_y", 0.1);
    this->get_parameter("crater_scale_y", crater_scale_y_);
    this->declare_parameter<float>("crater_scale_z", 0.2);
    this->get_parameter("crater_scale_z", crater_scale_z_);
    this->declare_parameter<float>("crater_r", 0.0);
    this->get_parameter("crater_r", crater_r_);
    this->declare_parameter<float>("crater_g", 1.0);
    this->get_parameter("crater_g", crater_g_);
    this->declare_parameter<float>("crater_b", 0.0);
    this->get_parameter("crater_b", crater_b_);

    makeCrater(crater1_, crater1id_, crater1x_, crater1y_);
    craters_msg_.markers.push_back(crater1_);

    makeCrater(crater2_, crater2id_, crater2x_, crater2y_);
    craters_msg_.markers.push_back(crater2_);
  }

  void MarkerVizNode::makeCrater(visualization_msgs::msg::Marker &crater, int id, float craterx, float cratery)
  {
    crater.header.frame_id = "map";
    crater.header.stamp = this->get_clock()->now();
    crater.ns = "craters";
    crater.id = id;
    crater.type = visualization_msgs::msg::Marker::CYLINDER;
    crater.action = visualization_msgs::msg::Marker::ADD;
    crater.pose.position.x = craterx;
    crater.pose.position.y = cratery;
    crater.pose.position.z = 0;
    crater.pose.orientation.x = 0.0;
    crater.pose.orientation.y = 0.0;
    crater.pose.orientation.z = 0.0;
    crater.pose.orientation.w = 1.0;
    crater.scale.x = crater_scale_x_;
    crater.scale.y = crater_scale_y_;
    crater.scale.z = crater_scale_z_;
    crater.color.a = 1.0; // Alpha controls transparency
    crater.color.r = crater_r_;
    crater.color.g = crater_g_;
    crater.color.b = crater_b_;
  }

  void MarkerVizNode::timerCallback()
  {

    // Publish both craters and the steering angle
    craters_pub_->publish(craters_msg_);
    arrow_pub_->publish(steer_angle_arrow_);
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