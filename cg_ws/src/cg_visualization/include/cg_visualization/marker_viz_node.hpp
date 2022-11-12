#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>

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
      rclcpp::Subscription<cg_msgs::msg::EncoderTelemetry>::SharedPtr enc_tele_sub_;

      /* Callbacks */
      void updateToolViz(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg);

      // Tool Position Visualization
      visualization_msgs::msg::Marker body, ground, tool1, tool2, viz_text;
      void toolPoseVizInit();

    }; // class MarkerVizNode
  } // namespace cg_visualization
} // namespace cg
