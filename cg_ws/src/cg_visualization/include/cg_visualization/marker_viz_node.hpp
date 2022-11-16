#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <math.h>

namespace cg
{
  namespace cg_visualization
  {
    class MarkerVizNode : public rclcpp::Node
    {

    public:
      MarkerVizNode();

    private:
      /* Publishers and Subscribers */
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tool_pose_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr steer_pose_pub_;
      rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr enc_tele_sub_;
      rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr act_cmd_sub_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

      /* Callbacks */
      void updateToolViz(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);
      void updateSteerViz(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
      void poseUpdateViz(const nav_msgs::msg::Odometry::SharedPtr msg);

      // Tool Position Visualization
      visualization_msgs::msg::Marker body, ground, tool1, tool2, tool_text;
      geometry_msgs::msg::PoseStamped steer;
      void toolPoseVizInit();

      float tool_pose_ = 0.0;
      double steer_pose_ = 0.0;
    }; // class MarkerVizNode
  } // namespace cg_visualization
} // namespace cg
