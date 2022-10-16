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
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr craters_pub_;

      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub_;
      rclcpp::Subscription<cg_msgs::msg::ActuatorState>::SharedPtr act_state_sub_;
      rclcpp::TimerBase::SharedPtr timer_; // For looping publish rate

      /* Message data */
      visualization_msgs::msg::MarkerArray craters_msg_;
      visualization_msgs::msg::Marker steer_angle_arrow_;

      /* Callbacks */
      void timerCallback(); // For looping publish in idle mode
      void actStateCallback(const cg_msgs::msg::ActuatorState::SharedPtr msg);

      /* Craters */
      visualization_msgs::msg::Marker crater1_;
      float crater1x_;
      float crater1y_;
      int crater1id_;

      visualization_msgs::msg::Marker crater2_;
      float crater2x_;
      float crater2y_;
      int crater2id_;

      // Crater vizualization
      float crater_scale_x_;
      float crater_scale_y_;
      float crater_scale_z_;
      float crater_r_;
      float crater_g_;
      float crater_b_;

      float arrow_scale_x_;
      float arrow_scale_y_;
      float arrow_scale_z_;
      float arrow_r_;
      float arrow_g_;
      float arrow_b_;

      /* Helpers */
      void makeCrater(visualization_msgs::msg::Marker& crater, int id, float craterx, float cratery);

    }; // class MarkerVizNode
  } // namespace cg_visualization
} // namespace cg

  // Publish markers from config file
  // Publish arrow from subscribed steer angle estimate