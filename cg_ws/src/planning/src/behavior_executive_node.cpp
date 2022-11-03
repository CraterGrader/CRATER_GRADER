#include "planning/behavior_executive_node.hpp"

namespace cg {
namespace planning {

  BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive_node")
  {
    /* Initialize publishers and subscribers */
    viz_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/viz/planning/current_path", 1);
    viz_goals_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/viz/planning/current_goals", 1);
    viz_agent_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/viz/planning/current_agent", 1);
    viz_curr_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/viz/planning/current_goal", 1);

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
    this->declare_parameter<int>("fsm_timer_callback_ms", 2000);
    this->get_parameter("fsm_timer_callback_ms", fsm_timer_callback_ms_);
    this->declare_parameter<int>("service_response_timeout_ms", 2000);
    this->get_parameter("service_response_timeout_ms", service_response_timeout_ms_);
    this->declare_parameter<int>("viz_timer_callback_ms", 500);
    this->get_parameter("viz_timer_callback_ms", viz_timer_callback_ms_);
    fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(fsm_timer_callback_ms_), std::bind(&BehaviorExecutive::fsmTimerCallback, this), fsm_timer_cb_group_);
    viz_timer_ = this->create_wall_timer(std::chrono::milliseconds(viz_timer_callback_ms_), std::bind(&BehaviorExecutive::vizTimerCallback, this), viz_timer_cb_group_);

    /* Load parameters */
    // Map parameters
    size_t map_height;
    size_t map_width;
    float map_resolution;
    float xTransform;
    float yTransform;
    std::string design_topo_filepath;
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
    this->declare_parameter<std::string>("design_topo_filepath", "/root/CRATER_GRADER/cg_ws/src/planning/config/50x50_zeros_height_map.csv");
    this->get_parameter("design_topo_filepath", design_topo_filepath);
    
    // Thresholds for reaching following goal
    this->declare_parameter<double>("thresh_pos", 1.0);
    this->get_parameter("thresh_pos", thresh_pos_);
    this->declare_parameter<double>("thresh_head", 1.0);
    this->get_parameter("thresh_head", thresh_head_);
    this->declare_parameter<double>("thresh_euclidean_replan", 1.0);
    this->get_parameter("thresh_euclidean_replan", thresh_euclidean_replan_);

    // Kinematic planner
    float goal_pose_distance_threshold;
    float goal_pose_yaw_threshold;
    float turn_radii_min;
    float turn_radii_max;
    float turn_radii_resolution;
    float max_trajectory_length;
    float trajectory_resolution;
    float pose_position_equality_threshold;
    float pose_yaw_equality_threshold;
    float topography_weight;
    float trajectory_heuristic_epsilon;
    float max_pose_equality_scalar;
    int pose_equality_scalar_iteration;
    this->declare_parameter<float>("goal_pose_distance_threshold", 0.00001);
    this->get_parameter("goal_pose_distance_threshold", goal_pose_distance_threshold);
    this->declare_parameter<float>("goal_pose_yaw_threshold", 0.00001);
    this->get_parameter("goal_pose_yaw_threshold", goal_pose_yaw_threshold);
    this->declare_parameter<float>("turn_radii_min", 1.6);
    this->get_parameter("turn_radii_min", turn_radii_min);
    this->declare_parameter<float>("turn_radii_max", 2.8);
    this->get_parameter("turn_radii_max", turn_radii_max);
    this->declare_parameter<float>("turn_radii_resolution", 0.4);
    this->get_parameter("turn_radii_resolution", turn_radii_resolution);
    this->declare_parameter<float>("max_trajectory_length", 0.4);
    this->get_parameter("max_trajectory_length", max_trajectory_length);
    this->declare_parameter<float>("trajectory_resolution", 0.05);
    this->get_parameter("trajectory_resolution", trajectory_resolution);
    this->declare_parameter<float>("pose_position_equality_threshold", 0.05);
    this->get_parameter("pose_position_equality_threshold", pose_position_equality_threshold);
    this->declare_parameter<float>("pose_yaw_equality_threshold", 0.0872665);
    this->get_parameter("pose_yaw_equality_threshold", pose_yaw_equality_threshold);
    this->declare_parameter<float>("topography_weight", 1.0);
    this->get_parameter("topography_weight", topography_weight);
    this->declare_parameter<float>("trajectory_heuristic_epsilon", 1.0);
    this->get_parameter("trajectory_heuristic_epsilon", trajectory_heuristic_epsilon);
    this->declare_parameter<float>("max_pose_equality_scalar", 1.0);
    this->get_parameter("max_pose_equality_scalar", max_pose_equality_scalar);
    this->declare_parameter<int>("pose_equality_scalar_iteration", 1000);
    this->get_parameter("pose_equality_scalar_iteration", pose_equality_scalar_iteration);

    // Exploration planner
    double exploration_min_dist_from_map_boundary;
    this->declare_parameter<double>("exploration_min_dist_from_map_boundary", 0.5);
    this->get_parameter("exploration_min_dist_from_map_boundary", exploration_min_dist_from_map_boundary);

    // Tool planner
    double design_blade_height, raised_blade_height;
    this->declare_parameter<double>("design_blade_height", 80.0);
    this->get_parameter<double>("design_blade_height", design_blade_height);
    this->declare_parameter<double>("raised_blade_height", 0.0);
    this->get_parameter<double>("raised_blade_height", raised_blade_height);

    // Velocity planner
    double constant_velocity;
    this->declare_parameter<double>("constant_velocity", 100.0);
    this->get_parameter<double>("constant_velocity", constant_velocity);

    transport_planner_ = std::make_unique<TransportPlanner>(TransportPlanner());
    exploration_planner_ = std::make_unique<ExplorationPlanner>(ExplorationPlanner(exploration_min_dist_from_map_boundary));
    kinematic_planner_ = std::make_unique<KinematicPlanner>(KinematicPlanner(
        goal_pose_distance_threshold,
        goal_pose_yaw_threshold,
        turn_radii_min,
        turn_radii_max,
        turn_radii_resolution,
        max_trajectory_length,
        trajectory_resolution,
        pose_position_equality_threshold,
        pose_yaw_equality_threshold,
        topography_weight,
        trajectory_heuristic_epsilon,
        max_pose_equality_scalar,
        pose_equality_scalar_iteration));
    tool_planner_ = std::make_unique<ToolPlanner>(ToolPlanner(design_blade_height, raised_blade_height));
    velocity_planner_ = std::make_unique<VelocityPlanner>(VelocityPlanner(constant_velocity));

    // Transport planner
    this->declare_parameter<float>("transport_threshold_z", 0.03);
    this->get_parameter("transport_threshold_z", transport_threshold_z_);
    this->declare_parameter<float>("thresh_max_assignment_distance", 0.7);
    this->get_parameter("thresh_max_assignment_distance", thresh_max_assignment_distance_);

    double last_pose_offset;
    this->declare_parameter<double>("last_pose_offset", 1.0);
    this->get_parameter("last_pose_offset", last_pose_offset);

    transport_planner_->setLastPoseOffset(last_pose_offset);

    // Exploration planner
    this->declare_parameter<float>("map_coverage_threshold", 0.01);
    this->get_parameter("map_coverage_threshold", map_coverage_threshold_);

    // Viz
    this->declare_parameter<double>("viz_planning_height", 0.0);
    this->get_parameter("viz_planning_height", viz_planning_height_);

    // Update map parameters
    current_height_map_.updateDimensions(map_height, map_width, map_resolution);
    bool design_height_map_initialized = design_height_map_.load_map_from_file(design_topo_filepath);

    // Validation parameters
    this->declare_parameter<float>("topology_equality_threshold", 0.03);
    this->get_parameter("topology_equality_threshold", topology_equality_threshold_);

    if (!design_height_map_initialized) {
      RCLCPP_FATAL(this->get_logger(), "Design map loading error");
      rclcpp::shutdown();
    }

    if ((design_height_map_.getHeight() != current_height_map_.getHeight()) \
          || (design_height_map_.getWidth() != current_height_map_.getWidth()) \
          || (design_height_map_.getResolution() != current_height_map_.getResolution())) {
      RCLCPP_FATAL(this->get_logger(), "Map dimensions do not align!\n    Site map <height, width, resolution, data.size()> <%ld, %ld, %f, %ld>\n    Design map <height, width, resolution, data.size()>: <%ld, %ld, %f, %ld>", current_height_map_.getHeight(), current_height_map_.getWidth(), current_height_map_.getResolution(), current_height_map_.getCellData().size(), design_height_map_.getHeight(), design_height_map_.getWidth(), design_height_map_.getResolution(), design_height_map_.getCellData().size());
      rclcpp::shutdown();
    }

    // Create pose of local map, assumed with no rotation
    local_map_relative_to_global_frame_ = create_pose2d(xTransform, yTransform, 0.0);
    global_map_relative_to_local_frame_ = create_pose2d(-xTransform, -yTransform, 0.0);

    // ---------------------
    // DEBUG
    // global_robot_pose_ = cg::planning::create_pose2d(1.0, 1.0, 0.0);
    // current_agent_pose_ = cg::planning::transformPose(global_robot_pose_, global_map_relative_to_local_frame_); // Convert pose to local map frame
    // ---------------------
  }

void BehaviorExecutive::fsmTimerCallback()
{

  // Run machine
  // std::cout << "~~~~~~~ Running machine..." << std::endl;
  std::cout << "~~~~~~~ Machine iteration" << std::endl;
  std::cout << "    Pre-Signal: " << fsm_.preSignalToString() << std::endl;
  std::cout << "      State L0: " << fsm_.currStateL0ToString() << std::endl;
  std::cout << "      State L1: " << fsm_.currStateL1ToString() << std::endl;
  switch (fsm_.getCurrStateL0())
  {
  case cg::planning::FSM::StateL0::READY:
    ready_.runState();
    break;
  case cg::planning::FSM::StateL0::UPDATE_MAP:
    map_updated_ = updateMapFromService(false);
    // Check that map was updated correctly
    RCLCPP_INFO(this->get_logger(), "Valid map update: %s", map_updated_ ? "true" : "false");
    // map_updated_ = true; // DEBUG: use to skip actual state checking
    update_map_.runState(map_updated_);
    break;
  case cg::planning::FSM::StateL0::SITE_WORK_DONE:
    site_work_done_.runState(current_height_map_, design_height_map_, topology_equality_threshold_);
    break;
  case cg::planning::FSM::StateL0::MAP_EXPLORED:
    map_explored_.runState(current_map_coverage_ratio_, map_coverage_threshold_);
    break;
  case cg::planning::FSM::StateL0::REPLAN_TRANSPORT:
    replan_transport_.runState();
    break;
  case cg::planning::FSM::StateL0::PLAN_TRANSPORT:
    plan_transport_.runState(*transport_planner_, current_height_map_, design_height_map_, current_seen_map_, transport_threshold_z_, thresh_max_assignment_distance_);
    state_l1_goal_poses_.clear();
    break;
  case cg::planning::FSM::StateL0::GET_TRANSPORT_GOALS:
    num_poses_before_ = current_goal_poses_.size(); // DEBUG
    get_transport_goals_.runState(current_goal_poses_, state_l1_goal_poses_, *transport_planner_, current_agent_pose_, current_height_map_);
    // ---------------------------------------
    // DEBUG
    std::cout << "  Init / updated goal poses: " << num_poses_before_ << " / " << current_goal_poses_.size() << std::endl;
    std::cout << "    Agent <x,y,yaw>: < " << current_agent_pose_.pt.x << ", " << current_agent_pose_.pt.y << ", " << current_agent_pose_.yaw << " >" << std::endl;
    
    for (size_t i =0; i < current_goal_poses_.size(); ++i){
      std::cout << "    Pose " << i<< " <x,y,yaw>: < " << current_goal_poses_[i].pt.x << ", " << current_goal_poses_[i].pt.y << ", " << current_goal_poses_[i].yaw << " >" << std::endl;
    }
    // ---------------------------------------
    break;
  case cg::planning::FSM::StateL0::PLAN_EXPLORATION:
    plan_exploration_.runState(*exploration_planner_, current_height_map_);
    state_l1_goal_poses_.clear();
    break;
  case cg::planning::FSM::StateL0::GET_EXPLORATION_GOALS:{
    get_exploration_goals_.runState(current_goal_poses_, state_l1_goal_poses_, *exploration_planner_, current_agent_pose_, current_height_map_);
    for (size_t i =0; i < current_goal_poses_.size(); ++i){
      std::cout << "    Exploration Pose <x,y,yaw>: "<< std::to_string(i) << " < " << current_goal_poses_[i].pt.x << ", " << current_goal_poses_[i].pt.y << ", " << current_goal_poses_[i].yaw << " >" << std::endl;
    }
    // ---------------------------------------
    // DEBUG
    // cg_msgs::msg::Pose2D manual_goal1 = create_pose2d(2.5, 1.5, 0.0);
    // cg_msgs::msg::Pose2D manual_goal2 = create_pose2d(1.0, 4.0, 3.14159);
    // current_goal_poses_.clear();
    // current_goal_poses_.push_back(manual_goal1);
    // current_goal_poses_.push_back(manual_goal2);
    // ---------------------------------------
    break;}
  case cg::planning::FSM::StateL0::GOALS_REMAINING:{
    goals_remaining_.runState(current_goal_poses_, current_goal_pose_);
    // ---------------------------------------
    // DEBUG
    std::cout << "    Num Goals: " << current_goal_poses_.size() << std::endl;
    std::cout << "    Current Goal Pose  <x,y,yaw>: < " << current_goal_pose_.pt.x << ", " << current_goal_pose_.pt.y << ", " << current_goal_pose_.yaw << " >" << std::endl;
    std::cout << "    Current Agent Pose <x,y,yaw>: < " << current_agent_pose_.pt.x << ", " << current_agent_pose_.pt.y << ", " << current_agent_pose_.yaw << " >" << std::endl;
    std::cout << "    ------------ Remaining goals: " << std::endl;
    for (size_t i = 0; i < current_goal_poses_.size(); ++i) {
      std::cout << "    Goal pose " << i << " <x,y,yaw>: < " << current_goal_poses_[i].pt.x << ", " << current_goal_poses_[i].pt.y << ", " << current_goal_poses_[i].yaw << " >" << std::endl;
    }
    // ---------------------------------------
    break;}
  case cg::planning::FSM::StateL0::GET_WORKSYSTEM_TRAJECTORY:
    // Get the trajectory
    // TODO: encapsulate these functions in to the state; e.g. make GetWorksystemTrajectory a friend class of BehaviorExecutive so GetWorksystemTrajectory can access service calls
    if (!calculated_trajectory_) {
      // Calculate path trajectory
      kinematic_planner_->generatePath(current_trajectory_.path, current_agent_pose_, current_goal_pose_, current_height_map_);

      // Calculate velocity trajectory
      velocity_planner_->generateVelocityTargets(current_trajectory_, current_agent_pose_, current_height_map_);

      // Calculate tool trajectory
      tool_planner_->setEnable(fsm_.getCurrStateL1() == FSM::StateL1::TRANSPORT);
      tool_planner_->generateToolTargets(current_trajectory_, current_agent_pose_, current_height_map_);
      
      // Convert to global frame
      for (unsigned int i = 0; i < current_trajectory_.path.size(); ++i) {
        cg_msgs::msg::Pose2D global_path_pose = cg::planning::transformPose(current_trajectory_.path[i], local_map_relative_to_global_frame_);
        current_trajectory_.path[i] = global_path_pose;
      }

      // DEBUG
      for (size_t i =0; i < current_trajectory_.path.size(); ++i){
        std::cout << "    Trajectory <x,y,yaw,v,tool>: " << std::to_string(i) << " < " << current_trajectory_.path[i].pt.x << ", " << current_trajectory_.path[i].pt.y << ", " << current_trajectory_.path[i].yaw << ", " << current_trajectory_.velocity_targets[i] << ", " << current_trajectory_.tool_positions[i] << " >" << std::endl;
      }
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
  case cg::planning::FSM::StateL0::FOLLOWING_TRAJECTORY:{
    bool keep_following = following_trajectory_.runState(current_agent_pose_, current_goal_pose_, thresh_pos_, thresh_head_, thresh_euclidean_replan_, current_trajectory_, global_robot_state_, global_robot_pose_);

    // Disable worksystem if goal is reached
    if (!keep_following) {
      enable_worksystem_ = false;
      worksystem_enabled_ = enableWorksystemService(enable_worksystem_, true);
    }

    break;}
  case cg::planning::FSM::StateL0::END_MISSION:
    end_mission_.runState();
    break;
  case cg::planning::FSM::StateL0::STOPPED:
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
  std::future_status status = result_future.wait_for(std::chrono::milliseconds(service_response_timeout_ms_));

  // Get response data and indicate if map was successfully updated or not
  if (status == std::future_status::ready) {
    if (verbose) {RCLCPP_INFO(this->get_logger(), "Received SiteMap response");}
    auto response = result_future.get();
    // Only update height map if data is valid
    if (response->success) {
      bool set_data = current_height_map_.setCellData(response->site_map.height_map);
      current_map_coverage_ratio_ = response->map_coverage_ratio;
      current_seen_map_.clear();
      for (auto seen: response->seen_map) {
        current_seen_map_.push_back(seen);
      }
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
  std::future_status status = result_future.wait_for(std::chrono::milliseconds(service_response_timeout_ms_));

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
  std::future_status status = result_future.wait_for(std::chrono::milliseconds(service_response_timeout_ms_));

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

  global_robot_pose_ = cg::planning::create_pose2d(global_robot_state_.pose.pose.position.x,
                                                                       global_robot_state_.pose.pose.position.y,
                                                                       global_robot_yaw);

  // Convert pose to local map frame
  current_agent_pose_ = cg::planning::transformPose(global_robot_pose_, global_map_relative_to_local_frame_);
}

void BehaviorExecutive::odomRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_robot_state_ = *msg;
}

void BehaviorExecutive::vizTimerCallback() {
  // Agent
  cg_msgs::msg::Pose2D global_agent_pose = cg::planning::transformPose(current_agent_pose_, local_map_relative_to_global_frame_);
  viz_agent_.pose.position.x = global_agent_pose.pt.x;
  viz_agent_.pose.position.y = global_agent_pose.pt.y;
  viz_agent_.pose.position.z = viz_planning_height_;

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
  viz_curr_goal_.pose.position.z = viz_planning_height_;

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
  for (cg_msgs::msg::Pose2D goal_pose : state_l1_goal_poses_) {
    cg_msgs::msg::Pose2D global_goal_pose = cg::planning::transformPose(goal_pose, local_map_relative_to_global_frame_);

    geometry_msgs::msg::Pose pose_single;
    pose_single.position.x = global_goal_pose.pt.x;
    pose_single.position.y = global_goal_pose.pt.y;
    pose_single.position.z = viz_planning_height_;

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
    // cg_msgs::msg::Pose2D global_traj_pose = cg::planning::transformPose(traj_pose, local_map_relative_to_global_frame_);
    cg_msgs::msg::Pose2D global_traj_pose = traj_pose; // DEBUG

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = global_traj_pose.pt.x;
    pose_stamped.pose.position.y = global_traj_pose.pt.y;
    pose_stamped.pose.position.z = viz_planning_height_;

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
