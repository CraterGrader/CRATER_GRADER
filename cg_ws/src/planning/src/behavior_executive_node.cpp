#include "planning/behavior_executive_node.hpp"

namespace cg {
namespace planning {

  BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive_node")
  {
    /* Initialize publishers and subscribers */

    // Timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&BehaviorExecutive::timerCallback, this));

    // Load parameters
    size_t map_height;
    size_t map_width;
    float map_resolution;
    this->declare_parameter<int>("height", 5);
    this->get_parameter("height", map_height);
    this->declare_parameter<int>("width", 5);
    this->get_parameter("width", map_width);
    this->declare_parameter<float>("resolution", 1.0);
    this->get_parameter("resolution", map_resolution);

    // Update map parameters
    height_map.updateDimensions(map_height, map_width, map_resolution);
  }

void BehaviorExecutive::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "height: %ld, width: %ld, res: %f", height_map.getHeight(), height_map.getWidth(), height_map.getResolution());
}


} // namespace planning
} // namespace cg
