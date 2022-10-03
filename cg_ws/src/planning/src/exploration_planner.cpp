#include <planning/common.hpp>
#include <planning/exploration_planner.hpp>

namespace cg {
namespace planning {

void ExplorationPlanner::planExploration(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map) {

  // For now, have a fixed set of waypoints
  // Drive in a circle, then back into a corner and drive forward toward the center

}

cg_msgs::msg::Pose2D ExplorationPlanner::getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::SiteMap &map) {

  // Check if we have reached the end of the exploration phase
  if (isExplorationDone()) {
    // TODO should this function return a special pose to indicate
    // exploration phase is done? Or will behavior executive call
    // isExplorationDone() to check?
    return agent_pose;
  }

  const auto &curr_goal = exploration_waypoints_[curr_waypoint_idx_];
  if (isGoalReached(agent_pose, curr_goal)) {
    return exploration_waypoints_[++curr_waypoint_idx_];
  }
  return curr_goal;
}


// TODO should this be a common function with shared thresholds?
bool ExplorationPlanner::isGoalReached(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg_msgs::msg::Pose2D &goal_pose) const {
  return isExplorationDone() || (
      cg::planning::euclidean_distance(agent_pose.pt, goal_pose.pt) <= xy_goal_thresh_ &&
      std::fabs(agent_pose.yaw - goal_pose.yaw) <= yaw_goal_thresh_
  );
}

}  // namespace planning
}  // namespace cg
