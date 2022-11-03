#ifndef PLANNING__BEHAVIOR_EXECUTIVE_HPP
#define PLANNING__BEHAVIOR_EXECUTIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mapping/map.hpp>
#include <cg_msgs/srv/site_map.hpp> // Service for receiving SiteMap height data
#include <cg_msgs/srv/update_trajectory.hpp>   // Service for updating current trajectory
#include <cg_msgs/srv/enable_worksystem.hpp> // Service to enable/disable worksystem controller
#include <nav_msgs/msg/odometry.hpp> // Callback for pose
#include <tf2/LinearMath/Matrix3x3.h> // For converting from nav_msgs quaternions to rpy

// Finite state machine and states
#include <planning/fsm/fsm.hpp>
#include <planning/fsm/ready.hpp>
#include <planning/fsm/update_map.hpp>
#include <planning/fsm/site_work_done.hpp>
#include <planning/fsm/map_explored.hpp>
#include <planning/fsm/replan_transport.hpp>
#include <planning/fsm/plan_transport.hpp>
#include <planning/fsm/get_transport_goals.hpp>
#include <planning/fsm/plan_exploration.hpp>
#include <planning/fsm/get_exploration_goals.hpp>
#include <planning/fsm/goals_remaining.hpp>
#include <planning/fsm/get_worksystem_trajectory.hpp>
#include <planning/fsm/following_trajectory.hpp>
#include <planning/fsm/stopped.hpp>

// Viz
#include <nav_msgs/msg/path.hpp> // For visualizing the current trajectory
#include <geometry_msgs/msg/pose_stamped.hpp> // For visualizing the current trajectory and agent pose
#include <geometry_msgs/msg/pose_array.hpp> // For visualizing the current goal poses
#include <tf2/LinearMath/Quaternion.h> // For visualizing the current goal poses

namespace cg {
namespace planning {

class BehaviorExecutive : public rclcpp::Node {

public:
  BehaviorExecutive();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr viz_goals_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr viz_agent_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr viz_curr_goal_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_robot_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot_state_sub_;

  /* Services */
  // Create callback groups for service call in timer: https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
  rclcpp::CallbackGroup::SharedPtr site_map_client_group_;
  rclcpp::CallbackGroup::SharedPtr update_trajectory_client_group_;
  rclcpp::CallbackGroup::SharedPtr enable_worksystem_client_group_;

  rclcpp::CallbackGroup::SharedPtr fsm_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr fsm_timer_; // For controlled looping fsm updates

  rclcpp::CallbackGroup::SharedPtr viz_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr viz_timer_; // For controlled looping viz updates

  rclcpp::Client<cg_msgs::srv::SiteMap>::SharedPtr site_map_client_;
  rclcpp::Client<cg_msgs::srv::UpdateTrajectory>::SharedPtr update_trajectory_client_;
  rclcpp::Client<cg_msgs::srv::EnableWorksystem>::SharedPtr enable_worksystem_client_;
  bool updateMapFromService(bool verbose);
  bool updateTrajectoryService(const cg_msgs::msg::Trajectory &current_trajectory, bool verbose);
  bool enableWorksystemService(const bool enable_worksystem, bool verbose);

  int fsm_timer_callback_ms_;
  long int viz_timer_callback_ms_;
  long int service_response_timeout_ms_;

  /* Message data */
  nav_msgs::msg::Path viz_path_;
  geometry_msgs::msg::PoseArray viz_goals_;
  geometry_msgs::msg::PoseStamped viz_agent_;
  geometry_msgs::msg::PoseStamped viz_curr_goal_;

  nav_msgs::msg::Odometry global_robot_state_;
  nav_msgs::msg::Odometry odom_robot_state_;

  /* Callbacks */
  void fsmTimerCallback(); // For looping publish
  void vizTimerCallback(); // For looping publish

  void globalRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odomRobotStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /* Important Objects */
  std::unique_ptr<cg::planning::TransportPlanner> transport_planner_;
  std::unique_ptr<cg::planning::ExplorationPlanner> exploration_planner_;
  std::unique_ptr<cg::planning::KinematicPlanner> kinematic_planner_;
  std::unique_ptr<cg::planning::ToolPlanner> tool_planner_;
  std::unique_ptr<cg::planning::VelocityPlanner> velocity_planner_;

  /* Variables */
  cg::mapping::Map<float> current_height_map_;
  cg_msgs::msg::Pose2D local_map_relative_to_global_frame_;
  cg_msgs::msg::Pose2D global_map_relative_to_local_frame_;

  // Params
  double thresh_pos_;
  double thresh_head_;
  float thresh_euclidean_replan_;

  // TODO: encapsulate these variables into their respective states, e.g. with friend classes/functions (for service calls)
  bool map_updated_ = false;
  bool updated_trajectory_ = false;
  bool calculated_trajectory_ = false;
  bool worksystem_enabled_ = false;
  size_t num_poses_before_; // DEBUG

  cg::mapping::Map<float> design_height_map_;
  float transport_threshold_z_;
  float thresh_max_assignment_distance_;
  double viz_planning_height_;

  std::vector<cg_msgs::msg::Pose2D> current_goal_poses_;
  std::vector<cg_msgs::msg::Pose2D> phase_goal_poses_;
  cg_msgs::msg::Pose2D current_goal_pose_; // Assumed to be in local map frame!
  cg_msgs::msg::Pose2D current_agent_pose_; // Assumed to be in local map frame!
  cg_msgs::msg::Pose2D global_robot_pose_; // Assumed to be in global map frame!
  bool enable_worksystem_ = false;
  cg_msgs::msg::Trajectory current_trajectory_; 

  // Create Finite State Machine
  cg::planning::FSM fsm_;

  // Initialize states
  cg::planning::Ready ready_;
  cg::planning::UpdateMap update_map_;
  cg::planning::SiteWorkDone site_work_done_;
  cg::planning::MapExplored map_explored_;
  cg::planning::ReplanTransport replan_transport_;
  cg::planning::PlanTransport plan_transport_; // Transport Planner
  cg::planning::GetTransportGoals get_transport_goals_;
  cg::planning::PlanExploration plan_exploration_; // Exploration Planner
  cg::planning::GetExplorationGoals get_exploration_goals_; 
  cg::planning::GoalsRemaining goals_remaining_;
  cg::planning::GetWorksystemTrajectory get_worksystem_trajectory_;
  cg::planning::FollowingTrajectory following_trajectory_;
  cg::planning::Stopped stopped_;

}; // class BehaviorExecutive

} // namespace planning
} // namespace cg

#endif // PLANNING__BEHAVIOR_EXECUTIVE_HPP

/**
 * TODO
 * [X] Agent callback
 * [ ] Check if goal is reached to transition out of following
 * [ ] Map checks
 * [ ] Make check for autonomous mode
 */