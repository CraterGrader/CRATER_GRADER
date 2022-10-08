#include "planning/behavior_executive_node.hpp"

namespace cg {
namespace planning {

  BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive_node")
  {
    /* Initialize publishers and subscribers */

    /* Initialize services */
    // Create reentrant callback group for service call in timer: https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create the service client, joined to the callback group
    site_map_client_ = this->create_client<cg_msgs::srv::SiteMap>("site_map_server", rmw_qos_profile_services_default, client_cb_group_);

    // Timer callback, joined to the callback group
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_callback_ms_), std::bind(&BehaviorExecutive::timerCallback, this), timer_cb_group_);

    // Load parameters
    size_t map_height;
    size_t map_width;
    float map_resolution;
    this->declare_parameter<int>("height", 50);
    this->get_parameter("height", map_height);
    this->declare_parameter<int>("width", 50);
    this->get_parameter("width", map_width);
    this->declare_parameter<float>("resolution", 0.1);
    this->get_parameter("resolution", map_resolution);

    // Update map parameters
    height_map_.updateDimensions(map_height, map_width, map_resolution);
  }

bool BehaviorExecutive::updateMapFromService(bool verbose = false) {
  // Create a request for the SiteMap service call
  auto request = std::make_shared<cg_msgs::srv::SiteMap::Request>();

  // Send request
  if (verbose) {RCLCPP_INFO(this->get_logger(), "Sending request");}
  auto result_future = site_map_client_->async_send_request(request);

  // Wait for response, until timeout at maximum
  std::future_status status = result_future.wait_for(std::chrono::seconds(service_response_timeout_sec_));

  // Get response data and indicate if map was successfully updated or not
  if (status == std::future_status::ready) {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "Received response");}
    auto response = result_future.get();
    // Only update height map if data is valid
    if (response->success) {
      bool set_data = height_map_.setCellData(response->site_map.height_map);
      return set_data;
    } 
    return false;
  }
  else {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "No response");}
    return false;
  }
}

void BehaviorExecutive::timerCallback()
{
  bool map_updated = updateMapFromService();
  // Check that map was updated correctly
  RCLCPP_INFO(this->get_logger(), "Valid map update: %s", map_updated ? "true" : "false");
  if (map_updated) {
    // TODO: Call planner modules
  }
}


} // namespace planning
} // namespace cg
