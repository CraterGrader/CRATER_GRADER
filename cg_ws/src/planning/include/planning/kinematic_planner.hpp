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

  // Construct Kinematic Plannercur_equality_scalar
  KinematicPlanner() : 
      goal_pose_distance_threshold_(std::vector<double>({0.1, 0.2, 0.5, 1.0})), 
      goal_pose_yaw_threshold_(deg2rad(5)), 
      turn_radii_min_(1.6f), 
      max_trajectory_length_(0.4f),
      trajectory_resolution_(0.05f),
      n_arms_(1),
      lattice_radii_scale_factor_(2.0),
      pose_position_equality_threshold_(0.05f),
      pose_yaw_equality_threshold_(deg2rad(5)),
      topography_weight_(1.0f),
      trajectory_heuristic_epsilon_(std::vector<double>({1.0, 2.0, 5.0, 10.0})),
      max_pose_equality_scalar_(2.0),
      pose_equality_scalar_iteration_(2000) {};

  // Construct Fully Parametric Planner
  KinematicPlanner(
    std::vector<double> goal_pose_distance_threshold,
    float goal_pose_yaw_threshold,
    float turn_radii_min,
    float max_trajectory_length,
    float trajectory_resolution,
    size_t n_arms,
    float lattice_radii_scale_factor,
    float pose_position_equality_threshold,
    float pose_yaw_equality_threshold,
    float topography_weight,
    std::vector<double> trajectory_heuristic_epsilon,
    float max_pose_equality_scalar,
    int pose_equality_scalar_iteration) : 
      goal_pose_distance_threshold_(goal_pose_distance_threshold), 
      goal_pose_yaw_threshold_(goal_pose_yaw_threshold), 
      turn_radii_min_(turn_radii_min), 
      max_trajectory_length_(max_trajectory_length),
      trajectory_resolution_(trajectory_resolution),
      n_arms_(n_arms),
      lattice_radii_scale_factor_(lattice_radii_scale_factor),
      pose_position_equality_threshold_(pose_position_equality_threshold),
      pose_yaw_equality_threshold_(pose_yaw_equality_threshold),
      topography_weight_(topography_weight),
      trajectory_heuristic_epsilon_(trajectory_heuristic_epsilon),
      max_pose_equality_scalar_(max_pose_equality_scalar),
      pose_equality_scalar_iteration_(pose_equality_scalar_iteration) {}; 

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
    const cg::mapping::Map<float> &map);

  // Truncates trajectory to closest pose to goal_pose, returns pose and index
  std::pair<cg_msgs::msg::Pose2D, int> getClosestTrajectoryPoseToGoal(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose) const;

  // Checks if trajectory_end_pose is within distance threshold of goal_pose
  bool samePoseWithinThresh(
    const cg_msgs::msg::Pose2D &trajectory_end_pose,
    const cg_msgs::msg::Pose2D &goal_pose) const;

  bool samePoseWithinThresh(
      const cg_msgs::msg::Pose2D &trajectory_end_pose,
      const cg_msgs::msg::Pose2D &goal_pose,
      const float pose_threshold,
      const float yaw_threshold) const;

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
  std::vector<float> trajectoriesHeuristic(
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &trajectories, 
    const cg_msgs::msg::Pose2D &goal_pose,
    double heuristic_epsilon) const;

  // Generate base lattice based on class parameters
  std::vector<std::vector<cg_msgs::msg::Pose2D>> generateBaseLattice(float max_trajectory_length) const;
  std::vector<cg_msgs::msg::Pose2D> generateLatticeArm(float turn_radius, bool forwards, bool right, float max_trajectory_length) const;

  // Getters
  std::vector<double> getGoalPoseDistanceThreshold() {return goal_pose_distance_threshold_;}
  float getGoalPoseYawThreshold() {return goal_pose_yaw_threshold_;}
  float getTurnRadiiMin() {return turn_radii_min_;}
  float getMaxTrajectoryLength() {return max_trajectory_length_;}
  float getTrajectoryResolution() {return trajectory_resolution_;}
  float getPosePositionEqualityThreshold() {return pose_position_equality_threshold_;}
  float getPoseYawEqualityThreshold() {return pose_yaw_equality_threshold_;}
  float getTopographyWeight() {return topography_weight_;}
  std::vector<double> getTrajectoryHeuristicEpsilon() {return trajectory_heuristic_epsilon_;}
  std::vector<std::vector<cg_msgs::msg::Pose2D>> getVizVisitedTrajectories() {return visited_trajectories;}

  // Setters
  void setGoalPoseDistanceThreshold(std::vector<double> val) {
    goal_pose_distance_threshold_ = val;
    return;
    }
  void setGoalPoseYawThreshold(float val) {
    goal_pose_yaw_threshold_ = val;
    return;
    }
  void setTurnRadiiMin(float val) {
    turn_radii_min_ = val;
    return;
    }
  void setMaxTrajectoryLength(float val) {
    max_trajectory_length_ = val;
    return;
    }
  void setTrajectoryResolution(float val) {
    trajectory_resolution_ = val;
    return;
    }
  void setPosePositionEqualityThreshold(float val) {
    pose_position_equality_threshold_ = val;
    return;
    }
  void setPoseYawEqualityThreshold(float val) {
    pose_yaw_equality_threshold_ = val;
    return;
    }
  void setTopographyWeight(float val) {
    topography_weight_ = val;
    return;
    }
  void setTrajectoryHeuristicEpsilon(std::vector<double> val) {
    trajectory_heuristic_epsilon_ = val;
    return;
    }

private:

  // Threshold to determine if trajectory end pose is a valid final pose
  std::vector<double> goal_pose_distance_threshold_;
  float goal_pose_yaw_threshold_;

  // Lattice Parameters
  float turn_radii_min_;
  float max_trajectory_length_;
  float trajectory_resolution_;
  size_t n_arms_;
  float lattice_radii_scale_factor_;

  // Pose equality thresholds
  float pose_position_equality_threshold_;
  float pose_yaw_equality_threshold_;

  // Cost Parameters
  float topography_weight_;
  std::vector<double> trajectory_heuristic_epsilon_;

  // A* parameters
  float max_pose_equality_scalar_;
  int pose_equality_scalar_iteration_;

  // Visualization parameters
  std::vector<std::vector<cg_msgs::msg::Pose2D>> visited_trajectories;
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
