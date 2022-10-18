#ifndef PLANNING__BEHAVIOR_EXECUTIVE_HPP
#define PLANNING__BEHAVIOR_EXECUTIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mapping/map.hpp>
#include <cg_msgs/srv/site_map.hpp> // Service for receiving SiteMap height data
#include <cg_msgs/srv/update_trajectory.hpp>   // Service for updating current trajectory
#include <cg_msgs/srv/enable_worksystem.hpp> // Service to enable/disable worksystem controller

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

  /* Services */
  // Create callback groups for service call in timer: https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
  rclcpp::CallbackGroup::SharedPtr site_map_client_group_;
  rclcpp::CallbackGroup::SharedPtr update_trajectory_client_group_;
  rclcpp::CallbackGroup::SharedPtr enable_worksystem_client_group_;

  rclcpp::CallbackGroup::SharedPtr fsm_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr fsm_timer_; // For controlled looping fsm updates

  rclcpp::CallbackGroup::SharedPtr viz_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr viz_timer_; // For controlled looping viz updates
  long int viz_timer_callback_ms_ = 500;

  rclcpp::Client<cg_msgs::srv::SiteMap>::SharedPtr site_map_client_;
  rclcpp::Client<cg_msgs::srv::UpdateTrajectory>::SharedPtr update_trajectory_client_;
  rclcpp::Client<cg_msgs::srv::EnableWorksystem>::SharedPtr enable_worksystem_client_;
  bool updateMapFromService(bool verbose);
  bool updateTrajectoryService(const cg_msgs::msg::Trajectory &current_trajectory, bool verbose);
  bool enableWorksystemService(const bool enable_worksystem, bool verbose);

  long int fsm_timer_callback_ms_ = 2000;
  long int service_response_timeout_sec_ = 2;

  /* Message data */
  nav_msgs::msg::Path viz_path_;
  geometry_msgs::msg::PoseArray viz_goals_;
  geometry_msgs::msg::PoseStamped viz_agent_;
  geometry_msgs::msg::PoseStamped viz_curr_goal_;

  /* Callbacks */
  void fsmTimerCallback(); // For looping publish
  void vizTimerCallback(); // For looping publish

  /* Important Objects */
  cg::planning::TransportPlanner transport_planner_;
  cg::planning::ExplorationPlanner exploration_planner_;
  cg::planning::KinematicPlanner kinematic_planner_;

  /* Variables */
  cg::mapping::Map<float> current_height_map_;
  cg_msgs::msg::Pose2D local_map_relative_to_global_frame_;

  // TODO: encapsulate these variables into their respective states, e.g. with friend classes/functions (for service calls)
  bool map_updated_ = false;
  bool updated_trajectory_ = false;
  bool calculated_trajectory_ = false;
  bool worksystem_enabled_ = false;
  size_t num_poses_before_; // DEBUG

  // DEBUG: test maps
  // std::vector<float> designTOPO_{0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                               0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<float> designTOPO_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  cg::mapping::Map<float> design_height_map_;
  float threshold_z_ = 0.03; // TODO: make this a config parameter

  std::vector<cg_msgs::msg::Pose2D> current_goal_poses_;
  cg_msgs::msg::Pose2D current_goal_pose_;
  cg_msgs::msg::Pose2D current_agent_pose_; // TODO: make callback so this gets updated, assumed to be in local map frame!
  bool enable_worksystem_ = false;
  cg_msgs::msg::Trajectory current_trajectory_; // TODO: actually use this
  std::vector<std::vector<cg_msgs::msg::Pose2D>> current_trajectories_;

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
