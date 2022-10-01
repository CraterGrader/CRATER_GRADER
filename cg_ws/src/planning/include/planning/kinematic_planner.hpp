#ifndef PLANNING__KINEMATIC_PLANNER_HPP
#define PLANNING__KINEMATIC_PLANNER_HPP

#include <vector>
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
    const cg::mapping::SiteMap &map);

  // Truncates trajectory to closest pose to goal_pose
  cg_msgs::msg::Pose2D getClosestTrajectoryPoseToGoal(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose);

  // Checks if trajectory_end_pose is within distance threshold of goal_pose
  bool samePoseWithThresh(
    const cg_msgs::msg::Pose2D &trajectory_end_pose,
    const cg_msgs::msg::Pose2D &goal_pose);

  // Creates a std::vector of lattice trajectories 
  // based on a base lattice type and current pose
  std::vector<std::vector<cg_msgs::msg::Pose2D>> transformLatticeToPose(
    const std::vector<cg_msgs::msg::Pose2D> &base_lattice,
    const cg_msgs::msg::Pose2D &current_pose);

  // Checks if trajectory is valid (eg. within worksite)
  bool isValidTrajectory(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory);

  // Calculates the total cost of the trajectory
  std::vector<float> calculateTrajectoryCost(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory,
    const cg::mapping::SiteMap &map);

  // Calculate heuristic associated with trajectory
  float trajectory_heuristic(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose);

  // Generate base lattice based on class parameters
  std::vector<std::vector<cg_msgs::msg::Pose2D>> generateBaseLattice();
  std::vector<cg_msgs::msg::Pose2D> generateLatticeArm(float turn_radius, bool forwards, bool right);

  // Threshold to determine if trajectory end pose is a valid final pose
  float goal_pose_distance_threshold;

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

} // namespace planning
} // namespace cg

#endif // PLANNING__KINEMATIC_PLANNER_HPP
