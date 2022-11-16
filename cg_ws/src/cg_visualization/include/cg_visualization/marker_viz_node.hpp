#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
#include <cg_msgs/msg/actuator_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
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
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tool_pose_pub_;
      rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr enc_tele_sub_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub_;

      /* Callbacks */
      void updateToolViz(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);
      void timerCallback(); // For looping publish in idle mode
      void actStateCallback(const cg_msgs::msg::ActuatorState::SharedPtr msg);

      // Tool Position Visualization
      visualization_msgs::msg::Marker body, ground, tool1, tool2, tool_text, steer_angle_arrow_;
      visualization_msgs::msg::MarkerArray tool_array;
      rclcpp::TimerBase::SharedPtr timer_; // For looping publish rate
      rclcpp::Subscription<cg_msgs::msg::ActuatorState>::SharedPtr act_state_sub_;
      float tool_pose_ = 0.0;

      void toolPoseVizInit();

      float arrow_scale_x_;
      float arrow_scale_y_;
      float arrow_scale_z_;
      float arrow_r_;
      float arrow_g_;
      float arrow_b_;

    }; // class MarkerVizNode
  } // namespace cg_visualization
} // namespace cg
