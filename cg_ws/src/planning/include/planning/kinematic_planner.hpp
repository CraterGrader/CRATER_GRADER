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

  // Construct Kinematic Planner
  // TODO: make these params
  KinematicPlanner() : 
      goal_pose_distance_threshold_(1e-5f), 
      turn_radii_min_(1.6f), 
      turn_radii_max_(2.8f), 
      turn_radii_resolution_(0.4f),
      max_trajectory_length_(0.4f),
      trajectory_resolution_(0.05f),
      pose_position_equality_threshold_(0.05f),
      pose_yaw_equality_threshold_(deg2rad(5)),
      topography_weight_(1.0f),
      trajectory_heuristic_epsilon_(1.0f) {};

  // Construct Fully Parametric Planner
  KinematicPlanner(
    float goal_pose_distance_threshold,
    float turn_radii_min,
    float turn_radii_max,
    float turn_radii_resolution,
    float max_trajectory_length,
    float trajectory_resolution,
    float pose_position_equality_threshold,
    float pose_yaw_equality_threshold,
    float topography_weight,
    float trajectory_heuristic_epsilon) : 
      goal_pose_distance_threshold_(goal_pose_distance_threshold), 
      turn_radii_min_(turn_radii_min), 
      turn_radii_max_(turn_radii_max), 
      turn_radii_resolution_(turn_radii_resolution),
      max_trajectory_length_(max_trajectory_length),
      trajectory_resolution_(trajectory_resolution),
      pose_position_equality_threshold_(pose_position_equality_threshold),
      pose_yaw_equality_threshold_(pose_yaw_equality_threshold),
      topography_weight_(topography_weight),
      trajectory_heuristic_epsilon_(trajectory_heuristic_epsilon) {}; 

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
  std::vector<float> trajectoriesHeuristic(
    const std::vector<std::vector<cg_msgs::msg::Pose2D>> &trajectories, 
    const cg_msgs::msg::Pose2D &goal_pose) const;

  // Generate base lattice based on class parameters
  std::vector<std::vector<cg_msgs::msg::Pose2D>> generateBaseLattice() const;
  std::vector<cg_msgs::msg::Pose2D> generateLatticeArm(float turn_radius, bool forwards, bool right) const;

  // Getters
  float getGoalPoseDistanceThreshold() {return goal_pose_distance_threshold_;}
  float getTurnRadiiMin() {return turn_radii_min_;}
  float getTurnRadiiMax() {return turn_radii_max_;}
  float getTurnRadiiResolutuion() {return turn_radii_resolution_;}
  float getMaxTrajectoryLength() {return max_trajectory_length_;}
  float getTrajectoryResolution() {return trajectory_resolution_;}
  float getPosePositionEqualityThreshold() {return pose_position_equality_threshold_;}
  float getPoseYawEqualityThreshold() {return pose_yaw_equality_threshold_;}
  float getTopographyWeight() {return topography_weight_;}
  float getTrajectoryHeuristicEpsilon() {return trajectory_heuristic_epsilon_;}

  // Setters
  void setGoalPoseDistanceThreshold(float val) {
    goal_pose_distance_threshold_ = val;
    return;
    }
  void setTurnRadiiMin(float val) {
    turn_radii_min_ = val;
    return;
    }
  void setTurnRadiiMax(float val) {
    turn_radii_max_ = val;
    return;
    }
  void setTurnRadiiResolutuion(float val) {
    turn_radii_resolution_ = val;
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
  void setTrajectoryHeuristicEpsilon(float val) {
    trajectory_heuristic_epsilon_ = val;
    return;
    }

private:

  // Threshold to determine if trajectory end pose is a valid final pose
  float goal_pose_distance_threshold_;

  // Lattice Parameters
  float turn_radii_min_;
  float turn_radii_max_;
  float turn_radii_resolution_;
  float max_trajectory_length_;
  float trajectory_resolution_;

  // Pose equality thresholds
  float pose_position_equality_threshold_;
  float pose_yaw_equality_threshold_;

  // Cost Parameters
  float topography_weight_;
  float trajectory_heuristic_epsilon_;

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

// Struct to hold parameters for kinematic planner configurations;
struct KinematicPlannerParams {
      float goal_pose_distance_threshold;
      float turn_radii_min;
      float turn_radii_max;
      float turn_radii_resolution;
      float max_trajectory_length;
      float trajectory_resolution;
      float pose_position_equality_threshold;
      float pose_yaw_equality_threshold;
      float topography_weight;
      float trajectory_heuristic_epsilon;

      KinematicPlanner(float goal_pose_distance_threshold,
        float turn_radii_min,
        float turn_radii_max,
        float turn_radii_resolution,
        float max_trajectory_length,
        float trajectory_resolution,
        float pose_position_equality_threshold,
        float pose_yaw_equality_threshold,
        float topography_weight,
        float trajectory_heuristic_epsilon) :
          goal_pose_distance_threshold(goal_pose_distance_threshold),
          turn_radii_min(turn_radii_min),
          turn_radii_max(turn_radii_max),
          turn_radii_resolution(turn_radii_resolution),
          max_trajectory_length(max_trajectory_length),
          trajectory_resolution(trajectory_resolution),
          pose_position_equality_threshold(pose_position_equality_threshold),
          pose_yaw_equality_threshold(pose_yaw_equality_threshold),
          topography_weight(topography_weight),
          trajectory_heuristic_epsilon(trajectory_heuristic_epsilon) {};
};

} // namespace planning
} // namespace cg

#endif // PLANNING__KINEMATIC_PLANNER_HPP
