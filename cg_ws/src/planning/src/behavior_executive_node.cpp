#include "planning/behavior_executive_node.hpp"

namespace cg {
namespace planning {

  BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive_node")
  {
    /* Initialize publishers and subscribers */
    viz_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/viz/current_path", 1);
    viz_goals_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/viz/current_goals", 1);
    viz_agent_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/viz/current_agent", 1);
    viz_curr_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/viz/current_goal", 1);

    global_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered/ekf_global_node", 1, std::bind(&BehaviorExecutive::globalRobotStateCallback, this, std::placeholders::_1));
    odom_robot_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered/ekf_odom_node", 1, std::bind(&BehaviorExecutive::odomRobotStateCallback, this, std::placeholders::_1));

    /* Initialize services */
    // Create reentrant callback group for service call in timer: https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
    site_map_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    update_trajectory_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    enable_worksystem_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    fsm_timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    viz_timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create the service client, joined to the callback group
    site_map_client_ = this->create_client<cg_msgs::srv::SiteMap>("site_map_server", rmw_qos_profile_services_default, site_map_client_group_);
    update_trajectory_client_ = this->create_client<cg_msgs::srv::UpdateTrajectory>("update_trajectory_server", rmw_qos_profile_services_default, update_trajectory_client_group_);
    enable_worksystem_client_ = this->create_client<cg_msgs::srv::EnableWorksystem>("enable_worksystem_server", rmw_qos_profile_services_default, enable_worksystem_client_group_);

    // Timer callback, joined to the callback group
    fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(fsm_timer_callback_ms_), std::bind(&BehaviorExecutive::fsmTimerCallback, this), fsm_timer_cb_group_);
    viz_timer_ = this->create_wall_timer(std::chrono::milliseconds(viz_timer_callback_ms_), std::bind(&BehaviorExecutive::vizTimerCallback, this), viz_timer_cb_group_);

    // Load parameters
    size_t map_height;
    size_t map_width;
    float map_resolution;
    float xTransform;
    float yTransform;
    this->declare_parameter<int>("height", 50);
    this->get_parameter("height", map_height);
    this->declare_parameter<int>("width", 50);
    this->get_parameter("width", map_width);
    this->declare_parameter<float>("resolution", 0.1);
    this->get_parameter("resolution", map_resolution);
    this->declare_parameter<float>("xTransform", 1.0);
    this->get_parameter("xTransform", xTransform);
    this->declare_parameter<float>("yTransform", 1.0);
    this->get_parameter("yTransform", yTransform);

    // Update map parameters
    current_height_map_.updateDimensions(map_height, map_width, map_resolution);
    design_height_map_.updateDimensions(map_height, map_width, map_resolution);
    design_height_map_.setCellData(designTOPO_);

    // Create pose of local map, assumed with no rotation
    local_map_relative_to_global_frame_ = create_pose2d(xTransform, yTransform, 0.0);
    global_map_relative_to_local_frame_ = create_pose2d(-xTransform, -yTransform, 0.0);
    current_agent_pose_ = create_pose2d(1.5, 0.5, 3.14159);
  }

void BehaviorExecutive::fsmTimerCallback()
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
    map_updated_ = updateMapFromService(false);
    // Check that map was updated correctly
    RCLCPP_INFO(this->get_logger(), "Valid map update: %s", map_updated_ ? "true" : "false");
    // map_updated_ = true; // DEBUG: use to skip actual state checking
    update_map_.runState(map_updated_);
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
    num_poses_before_ = current_goal_poses_.size(); // DEBUG
    get_transport_goals_.runState(current_goal_poses_, transport_planner_, current_agent_pose_, current_height_map_);
    std::cout << "  Init / updated goal poses: " << num_poses_before_ << " / " << current_goal_poses_.size() << std::endl;
    std::cout << "    Agent <x,y,yaw>: < " << current_agent_pose_.pt.x << ", " << current_agent_pose_.pt.y << ", " << current_agent_pose_.yaw << " >" << std::endl;
    std::cout << "    Pose1 <x,y,yaw>: < " << current_goal_poses_[0].pt.x << ", " << current_goal_poses_[0].pt.y << ", " << current_goal_poses_[0].yaw << " >" << std::endl;
    std::cout << "    Pose2 <x,y,yaw>: < " << current_goal_poses_[1].pt.x << ", " << current_goal_poses_[1].pt.y << ", " << current_goal_poses_[1].yaw << " >" << std::endl;
    std::cout << "    Pose3 <x,y,yaw>: < " << current_goal_poses_[2].pt.x << ", " << current_goal_poses_[2].pt.y << ", " << current_goal_poses_[2].yaw << " >" << std::endl;
    break;
  case cg::planning::FSM::State::PLAN_EXPLORATION:
    plan_exploration_.runState(exploration_planner_, current_height_map_);
    break;
  case cg::planning::FSM::State::GET_EXPLORATION_GOALS:
    get_exploration_goals_.runState(current_goal_poses_, exploration_planner_, current_agent_pose_, current_height_map_);
    for (size_t i =0; i < current_goal_poses_.size(); ++i){
      std::cout << "    Exploration Pose <x,y,yaw>: "<< std::to_string(i) << " < " << current_goal_poses_[i].pt.x << ", " << current_goal_poses_[i].pt.y << ", " << current_goal_poses_[i].yaw << " >" << std::endl;
    }
    break;
  case cg::planning::FSM::State::GOALS_REMAINING:
    goals_remaining_.runState(current_goal_poses_, current_goal_pose_);
    std::cout << "    Current Goal Pose  <x,y,yaw>: < " << current_goal_pose_.pt.x << ", " << current_goal_pose_.pt.y << ", " << current_goal_pose_.yaw << " >" << std::endl;
    std::cout << "    Current Agent Pose <x,y,yaw>: < " << current_agent_pose_.pt.x << ", " << current_agent_pose_.pt.y << ", " << current_agent_pose_.yaw << " >" << std::endl;
    break;
  case cg::planning::FSM::State::GET_WORKSYSTEM_TRAJECTORY:
    // Get the trajectory
    // get_worksystem_trajectory_.runStateMultiGoal(kinematic_planner_, current_goal_poses_, current_trajectories_, current_agent_pose_, current_height_map_);
    // for (size_t i = 0; i < current_trajectories_.size(); ++i) {
    //   std::cout << "Trajectory " << std::to_string(i) << std::endl;
    //   for (cg_msgs::msg::Pose2D pose: current_trajectories_[i]) {
    //     std::cout << "< " << pose.pt.x << ", " << pose.pt.y << ", " << pose.yaw << " >" << std::endl;
    //   }
    // }

    // TODO: encapsulate these functions in to the state; e.g. make GetWorksystemTrajectory a friend class of BehaviorExecutive so GetWorksystemTrajectory can access service calls
    if (!calculated_trajectory_) {
      // Update path trajectory
      kinematic_planner_.generatePath(current_trajectory_.path, current_agent_pose_, current_goal_pose_, current_height_map_);

      // TODO: update velocity trajectory
      velocity_planner_.generateVelocityTargets(current_trajectory_, current_agent_pose_, current_height_map_);

      // TODO: update tool trajectory
      tool_planner_.generateToolTargets(current_trajectory_, current_agent_pose_, current_height_map_);

      calculated_trajectory_ = true;
    }

    if (calculated_trajectory_) {
      // Send the trajectory to the controller
      updated_trajectory_ = updateTrajectoryService(current_trajectory_, true);
      if (updated_trajectory_) {
        // The controller trajectory was updated, so enable worksystem
        enable_worksystem_ = true;
        worksystem_enabled_ = enableWorksystemService(enable_worksystem_, true);
      }
    }
    
    // Update shared current state and the precursing signal if worksystem is now enabled
    get_worksystem_trajectory_.runState(worksystem_enabled_, updated_trajectory_, calculated_trajectory_);
    break;
  case cg::planning::FSM::State::FOLLOWING_TRAJECTORY:
    following_trajectory_.runState(current_agent_pose_, current_goal_pose_, thresh_pos_, thresh_head_);
    break;
  case cg::planning::FSM::State::STOPPED:
    // Stop the worksystem
    enable_worksystem_ = false;
    enableWorksystemService(enable_worksystem_, true);

    // Run state
    stopped_.runState();
    break;
  default:
    std::cout << "~ ~ ~ ~ ! Invalid State !" << std::endl;
    break;
  }
  // std::cout << "~~~~~~~ Machine done!" << std::endl;
}

bool BehaviorExecutive::updateMapFromService(bool verbose = false) {
  // Create a request for the SiteMap service call
  auto request = std::make_shared<cg_msgs::srv::SiteMap::Request>();

  // Send request
  if (verbose) {RCLCPP_INFO(this->get_logger(), "Sending SiteMap request");}
  auto result_future = site_map_client_->async_send_request(request);

  // Wait for response, until timeout at maximum
  std::future_status status = result_future.wait_for(std::chrono::seconds(service_response_timeout_sec_));

  // Get response data and indicate if map was successfully updated or not
  if (status == std::future_status::ready) {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "Received SiteMap response");}
    auto response = result_future.get();
    // Only update height map if data is valid
    if (response->success) {
      bool set_data = current_height_map_.setCellData(response->site_map.height_map);
      return set_data;
    } 
    return false;
  }
  else {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "No SiteMap response");}
    return false;
  }
}

bool BehaviorExecutive::updateTrajectoryService(const cg_msgs::msg::Trajectory &current_trajectory, bool verbose = false)
{
  // Create a request for the UpdateTrajectory service call
  auto request = std::make_shared<cg_msgs::srv::UpdateTrajectory::Request>();
  request->trajectory = current_trajectory;

  // Send request
  if (verbose) {RCLCPP_INFO(this->get_logger(), "Sending UpdateTrajectory request");}
  auto result_future = update_trajectory_client_->async_send_request(request);

  // Wait for response, until timeout at maximum
  std::future_status status = result_future.wait_for(std::chrono::seconds(service_response_timeout_sec_));

  // Get response data and indicate if map was successfully updated or not
  if (status == std::future_status::ready) {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "Received UpdateTrajectory response");}
    auto response = result_future.get();
    // Report response status
    return response->updated_trajectory;
  }
  else {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "No UpdateTrajectory response");}
    return false;
  }
}

bool BehaviorExecutive::enableWorksystemService(const bool enable_worksystem, bool verbose = false) {
  // Create a request for the EnableWorksystem service call
  auto request = std::make_shared<cg_msgs::srv::EnableWorksystem::Request>();
  request->enable_worksystem = enable_worksystem;

  // Send request
  if (verbose) {RCLCPP_INFO(this->get_logger(), "Sending EnableWorksystem request");}
  auto result_future = enable_worksystem_client_->async_send_request(request);

  // Wait for response, until timeout at maximum
  std::future_status status = result_future.wait_for(std::chrono::seconds(service_response_timeout_sec_));

  // Get response data and indicate if map was successfully updated or not
  if (status == std::future_status::ready) {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "Received EnableWorksystem response");}
    auto response = result_future.get();
    // Report response status
    return response->worksystem_enabled;
  }
  else {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "No EnableWorksystem response");}
    return false;
  }
}

void BehaviorExecutive::globalRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Read in the message
  global_robot_state_ = *msg;

  // Convert to pose
  tf2::Quaternion q(global_robot_state_.pose.pose.orientation.x,
                    global_robot_state_.pose.pose.orientation.y,
                    global_robot_state_.pose.pose.orientation.z,
                    global_robot_state_.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double global_robot_roll, global_robot_pitch, global_robot_yaw;
  m.getRPY(global_robot_roll, global_robot_pitch, global_robot_yaw);

  cg_msgs::msg::Pose2D global_robot_pose = cg::planning::create_pose2d(global_robot_state_.pose.pose.position.x,
                                                                       global_robot_state_.pose.pose.position.x,
                                                                       global_robot_yaw);

  // Convert pose to local map frame
  current_agent_pose_ = cg::planning::transformPose(global_robot_pose, global_map_relative_to_local_frame_);
}

void BehaviorExecutive::odomRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_robot_state_ = *msg;
}

void BehaviorExecutive::vizTimerCallback() {
  // Agent
  cg_msgs::msg::Pose2D global_agent_pose = cg::planning::transformPose(current_agent_pose_, local_map_relative_to_global_frame_);
  viz_agent_.pose.position.x = global_agent_pose.pt.x;
  viz_agent_.pose.position.y = global_agent_pose.pt.y;

  tf2::Quaternion q_agent;
  q_agent.setRPY(0, 0, global_agent_pose.yaw);
  viz_agent_.pose.orientation.x = q_agent.x();
  viz_agent_.pose.orientation.y = q_agent.y();
  viz_agent_.pose.orientation.z = q_agent.z();
  viz_agent_.pose.orientation.w = q_agent.w();

  viz_agent_.header.stamp = this->get_clock()->now();
  viz_agent_.header.frame_id = "map";

  viz_agent_pub_->publish(viz_agent_);

  // Current goal
  cg_msgs::msg::Pose2D global_curr_goal_pose = cg::planning::transformPose(current_goal_pose_, local_map_relative_to_global_frame_);
  viz_curr_goal_.pose.position.x = global_curr_goal_pose.pt.x;
  viz_curr_goal_.pose.position.y = global_curr_goal_pose.pt.y;

  tf2::Quaternion q_curr_goal;
  q_curr_goal.setRPY(0, 0, global_curr_goal_pose.yaw);
  viz_curr_goal_.pose.orientation.x = q_curr_goal.x();
  viz_curr_goal_.pose.orientation.y = q_curr_goal.y();
  viz_curr_goal_.pose.orientation.z = q_curr_goal.z();
  viz_curr_goal_.pose.orientation.w = q_curr_goal.w();

  viz_curr_goal_.header.stamp = this->get_clock()->now();
  viz_curr_goal_.header.frame_id = "map";

  viz_curr_goal_pub_->publish(viz_curr_goal_);

  // Goal poses
  viz_goals_.poses.clear();
  for (cg_msgs::msg::Pose2D goal_pose : current_goal_poses_) {
    cg_msgs::msg::Pose2D global_goal_pose = cg::planning::transformPose(goal_pose, local_map_relative_to_global_frame_);

    geometry_msgs::msg::Pose pose_single;
    pose_single.position.x = global_goal_pose.pt.x;
    pose_single.position.y = global_goal_pose.pt.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, global_goal_pose.yaw);
    pose_single.orientation.x = q.x();
    pose_single.orientation.y = q.y();
    pose_single.orientation.z = q.z();
    pose_single.orientation.w = q.w();

    viz_goals_.poses.push_back(pose_single);
  }
  viz_goals_.header.stamp = this->get_clock()->now();
  viz_goals_.header.frame_id = "map";
  viz_goals_pub_->publish(viz_goals_);

  // Trajectory
  viz_path_.poses.clear();
  for (cg_msgs::msg::Pose2D traj_pose : current_trajectory_.path) {
    cg_msgs::msg::Pose2D global_traj_pose = cg::planning::transformPose(traj_pose, local_map_relative_to_global_frame_);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = global_traj_pose.pt.x;
    pose_stamped.pose.position.y = global_traj_pose.pt.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, global_traj_pose.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    pose_stamped.header.stamp = this->get_clock()->now();
    pose_stamped.header.frame_id = "map";

    viz_path_.poses.push_back(pose_stamped);
  }
  viz_path_.header.stamp = this->get_clock()->now();
  viz_path_.header.frame_id = "map";

  viz_path_pub_->publish(viz_path_);
}

} // namespace planning
} // namespace cg
