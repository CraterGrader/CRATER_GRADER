#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/actuator_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

      // rclcpp::Subscription<cg_msgs::msg::ActuatorState>::SharedPtr act_state_sub_;
      rclcpp::TimerBase::SharedPtr timer_; // For looping publish rate

      /* Message data */
      visualization_msgs::msg::MarkerArray markers_msg_;
      visualization_msgs::msg::Marker crater1_;
      visualization_msgs::msg::Marker steer_angle_arrow_;

      /* Callbacks */
      void timerCallback(); // For looping publish in idle mode

      /* Helpers */


    }; // class MarkerVizNode
  } // namespace cg_visualization
} // namespace cg

  // Publish markers from config file
  // Publish arrow from subscribed steer angle estimate