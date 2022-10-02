#ifndef PLANNING__KINEMATIC_PLANNER_HPP
#define PLANNING__KINEMATIC_PLANNER_HPP

#include <vector>
#include <queue>
#include <cmath.h>
#include <planning/common.hpp>
#include <mapping/site_map.hpp>
#include <cg_msgs/msg/pose2_d.hpp>


namespace cg {
namespace planning {

class KinematicPlanner {

public:
  // Updates the path field in-place
  void generatePath(
    std::vector<cg_msgs::msg::Pose2D> &path,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg_msgs::msg::Pose2D &goal_pose,
    const cg::mapping::SiteMap &map);
  
  // Performs a lattice astar search agent in map environment
  std::vector<cg_msgs::msg::Pose2D> latticeAStarSearch(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg_msgs::msg::Pose2D &goal_pose,
    const cg::mapping::SiteMap &map,
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice);

  // Check if two poses are close enough according to class thresholds
  bool posesWithinThresh(
    const cg_msgs::msg::Pose2D &pose,
    const std::vector<cg_msgs::msg::Pose2D> &goal_pose);

  // Truncates trajectory to closest pose to goal_pose, returns pose and index
  std::pair<cg_msgs::msg::Pose2D, int> getClosestTrajectoryPoseToGoal(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose);

  // Checks if trajectory_end_pose is within distance threshold of goal_pose
  bool samePoseWithinThresh(
    const cg_msgs::msg::Pose2D &trajectory_end_pose,
    const cg_msgs::msg::Pose2D &goal_pose);

  // Creates a std::vector of lattice trajectories 
  // based on a base lattice type and current pose
  std::vector<std::vector<cg_msgs::msg::Pose2D>> transformLatticeToPose(
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice,
    const cg_msgs::msg::Pose2D &current_pose);

  // Checks if trajectory is valid (eg. within worksite)
  bool isValidTrajectory(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg::mapping::SiteMap &map);

  // Calculates the total cost of the topography for trajectory
  std::vector<float> calculateTopographyCost(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory,
    const cg::mapping::SiteMap &map);

  // Calculate heuristic associated with trajectories
  std::vector<float> trajectories_heuristic(
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &trajectories, 
    const cg_msgs::msg::Pose2D &goal_pose);

  // Generate base lattice based on class parameters
  std::vector<std::vector<cg_msgs::msg::Pose2D>> generateBaseLattice();
  std::vector<cg_msgs::msg::Pose2D> generateLatticeArm(float turn_radius, bool forwards, bool right);

  // Threshold to determine if trajectory end pose is a valid final pose
  float goal_pose_distance_threshold;

  // Pose equality thresholds
  float pose_position_equality_threshold;
  float pose_yaw_equality_threshold;

  // Lattice Parameters
  float turn_radii_min;
  float turn_radii_max;
  float turn_radii_resolution;
  float max_trajectory_length;
  float trajectory_resolution;

  // Cost Parameters
  float topography_weight;
  float trajectory_heuristic_epsilon;

};

struct AStarNode {

  float g_cost;
  int idx;
  int parent_idx;
  cg_msgs::msg::Pose2D pose;
  std::vector<cg_msgs::msg::Pose2D> trajectory;

  AStarNode(g_cost, idx, parent_idx, pose, trajectory) :
    g_cost(g_cost), idx(idx), parent_idx(idx), pose(pose), trajectory(trajectory) {};

};

} // namespace planning
} // namespace cg

#endif // PLANNING__KINEMATIC_PLANNER_HPP
