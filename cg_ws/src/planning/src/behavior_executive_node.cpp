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
    current_height_map_.updateDimensions(map_height, map_width, map_resolution);
    design_height_map_.updateDimensions(map_height, map_width, map_resolution);
    design_height_map_.setCellData(designTOPO_);
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
      bool set_data = current_height_map_.setCellData(response->site_map.height_map);
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

  // Run machine
  // std::cout << "~~~~~~~ Running machine..." << std::endl;
  std::cout << "~~~~~~~ Machine iteration" << std::endl;
  std::cout << "    Pre-Signal: " << fsm_.preSignalToString() << std::endl;
  std::cout << "         State: " << fsm_.currStateToString() << std::endl;
  switch (fsm_.getCurrState())
  {
  case cg::planning::FSM::State::READY:
    ready_.runState();
    break;
  case cg::planning::FSM::State::UPDATE_MAP:
    map_updated_ = updateMapFromService();
    // Check that map was updated correctly
    RCLCPP_INFO(this->get_logger(), "Valid map update: %s", map_updated_ ? "true" : "false");
    // if (map_updated) {
    //   // TODO: Call planner modules
    // }
    update_map_.runState();
    break;
  case cg::planning::FSM::State::SITE_WORK_DONE:
    site_work_done_.runState();
    break;
  case cg::planning::FSM::State::MAP_EXPLORED:
    map_explored_.runState();
    break;
  case cg::planning::FSM::State::REPLAN_TRANSPORT:
    replan_transport_.runState();
    break;
  case cg::planning::FSM::State::PLAN_TRANSPORT:
    plan_transport_.runState(transport_planner_, current_height_map_, design_height_map_, threshold_z_);
    break;
  case cg::planning::FSM::State::GET_TRANSPORT_GOALS:
    current_agent_pose_ = cg::planning::create_pose2d(1.0, 1.0, 1.0);
    num_poses_before_ = current_goal_poses_.size(); // DEBUG
    get_transport_goals_.runState(current_goal_poses_, transport_planner_, current_agent_pose_, current_height_map_);
    std::cout << "  Init / updated goal poses: " << num_poses_before_ << " / " << current_goal_poses_.size() << std::endl;
    std::cout << "    Agent <x,y,yaw>: < " << current_agent_pose_.pt.x << ", " << current_agent_pose_.pt.y << ", " << current_agent_pose_.yaw << " >" << std::endl;
    std::cout << "    Pose1 <x,y,yaw>: < " << current_goal_poses_[0].pt.x << ", " << current_goal_poses_[0].pt.y << ", " << current_goal_poses_[0].yaw << " >" << std::endl;
    std::cout << "    Pose2 <x,y,yaw>: < " << current_goal_poses_[1].pt.x << ", " << current_goal_poses_[1].pt.y << ", " << current_goal_poses_[1].yaw << " >" << std::endl;
    std::cout << "    Pose3 <x,y,yaw>: < " << current_goal_poses_[2].pt.x << ", " << current_goal_poses_[2].pt.y << ", " << current_goal_poses_[2].yaw << " >" << std::endl;
    break;
  case cg::planning::FSM::State::PLAN_EXPLORATION:
    plan_exploration_.runState();
    break;
  case cg::planning::FSM::State::GET_EXPLORATION_GOALS:
    get_exploration_goals_.runState(current_goal_poses_, exploration_planner_, current_agent_pose_, current_height_map_);
    for (size_t i =0; i < current_goal_poses_.size(); ++i){
      std::cout << "    Exploration Pose <x,y,yaw>: "<< std::to_string(i) << " < " << current_goal_poses_[i].pt.x << ", " << current_goal_poses_[i].pt.y << ", " << current_goal_poses_[i].yaw << " >" << std::endl;
    }
    break;
  case cg::planning::FSM::State::GOALS_REMAINING:
    goals_remaining_.runState();
    break;
  case cg::planning::FSM::State::GET_WORKSYSTEM_TRAJECTORY:
    get_worksystem_trajectory_.runState(kinematic_planner_, current_goal_poses_, current_trajectories_, current_agent_pose_, current_height_map_);
    for (size_t i = 0; i < current_trajectories_.size(); ++i) {
      std::cout << "Trajectory " << std::to_string(i) << std::endl;
      for (cg_msgs::msg::Pose2D pose: current_trajectories_[i]) {
        std::cout << "< " << pose.pt.x << ", " << pose.pt.y << ", " << pose.yaw << " >" << std::endl;
      }
    }
    break;
  case cg::planning::FSM::State::STOPPED:
    stopped_.runState();
    break;
  default:
    std::cout << "~ ~ ~ ~ ! Invalid State !" << std::endl;
    break;
  }
  // std::cout << "~~~~~~~ Machine done!" << std::endl;
}


} // namespace planning
} // namespace cg
