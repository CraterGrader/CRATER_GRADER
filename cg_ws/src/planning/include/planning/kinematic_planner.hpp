#ifndef PLANNING__KINEMATIC_PLANNER_HPP
#define PLANNING__KINEMATIC_PLANNER_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <planning/common.hpp>
#include <mapping/map.hpp>


namespace cg {
namespace planning {

class KinematicPlanner {

public:

  // Threshold to determine if trajectory end pose is a valid final pose
  float goal_pose_distance_threshold;

  // Lattice Parameters
  float turn_radii_min;
  float turn_radii_max;
  float turn_radii_resolution;
  float max_trajectory_length;
  float trajectory_resolution;

  // Pose equality thresholds
  float pose_position_equality_threshold;
  float pose_yaw_equality_threshold;

  // Cost Parameters
  float topography_weight;
  float trajectory_heuristic_epsilon;

  // Construct Kinematic Planner
  KinematicPlanner() : 
      goal_pose_distance_threshold(1e-5), 
      turn_radii_min(0.8), 
      turn_radii_max(1.6), 
      turn_radii_resolution(0.4),
      max_trajectory_length(0.4),
      trajectory_resolution(0.05),
      pose_position_equality_threshold(0.05),
      pose_yaw_equality_threshold(deg2rad(5)),
      topography_weight(1),
      trajectory_heuristic_epsilon(1) {};

  // Updates the path field in-place
  void generatePath(
    std::vector<cg_msgs::msg::Pose2D> &path,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg_msgs::msg::Pose2D &goal_pose,
    const cg::mapping::Map<float> &map);
  
  // Performs a lattice astar search agent in map environment
  std::vector<cg_msgs::msg::Pose2D> latticeAStarSearch(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg_msgs::msg::Pose2D &goal_pose,
    const cg::mapping::Map<float> &map,
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice) const;

  // Truncates trajectory to closest pose to goal_pose, returns pose and index
  std::pair<cg_msgs::msg::Pose2D, int> getClosestTrajectoryPoseToGoal(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose) const;

  // Checks if trajectory_end_pose is within distance threshold of goal_pose
  bool samePoseWithinThresh(
    const cg_msgs::msg::Pose2D &trajectory_end_pose,
    const cg_msgs::msg::Pose2D &goal_pose) const;

  // Creates a std::vector of lattice trajectories 
  // based on a base lattice type and current pose
  std::vector<std::vector<cg_msgs::msg::Pose2D>> transformLatticeToPose(
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice,
    const cg_msgs::msg::Pose2D &current_pose) const;

  // Checks if trajectory is valid (eg. within worksite)
  bool isValidTrajectory(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg::mapping::Map<float> &map) const;

  // Calculates the total cost of the topography for trajectory
  float calculateTopographyCost(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory,
    const cg::mapping::Map<float> &map) const;

  // Calculate heuristic associated with trajectories
  std::vector<float> trajectories_heuristic(
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &trajectories, 
    const cg_msgs::msg::Pose2D &goal_pose) const;

  // Generate base lattice based on class parameters
  std::vector<std::vector<cg_msgs::msg::Pose2D>> generateBaseLattice() const;
  std::vector<cg_msgs::msg::Pose2D> generateLatticeArm(float turn_radius, bool forwards, bool right) const;

};

struct AStarNode {

  float g_cost;
  int idx;
  int parent_idx;
  cg_msgs::msg::Pose2D pose;
  std::vector<cg_msgs::msg::Pose2D> trajectory;

  AStarNode(
    float g_cost, 
    int idx, 
    int parent_idx, 
    cg_msgs::msg::Pose2D pose, 
    std::vector<cg_msgs::msg::Pose2D> trajectory) :
      g_cost(g_cost), 
      idx(idx), 
      parent_idx(parent_idx), 
      pose(pose), 
      trajectory(trajectory) {};

    bool operator<(const AStarNode& rhs) const {
      return g_cost < rhs.g_cost;
    }



};

} // namespace planning
} // namespace cg

#endif // PLANNING__KINEMATIC_PLANNER_HPP
