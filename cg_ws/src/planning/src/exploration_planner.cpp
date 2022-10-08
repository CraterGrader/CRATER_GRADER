#include <planning/common.hpp>
#include <planning/exploration_planner.hpp>

namespace cg {
namespace planning {

void ExplorationPlanner::planExploration(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map) {

  // For now, have a fixed set of waypoints
  // Drive in a circle, then back into a corner and drive forward toward the center
  const double &h = map.getHeight()*map.getResolution();
  const double &w = map.getWidth()*map.getResolution();
  const double xc = w/2.0, yc = h/2.0;

  // Note: for now we assume driving a circle (constant radius)
  // If the map is non-square, then technically we should be driving an ellipse
  double r = std::min(h,w)/2.0 - min_dist_from_map_boundary_;
  // Generate waypoints going clockwise around a circle
  std::vector<int> thetas_deg = {
    270, 225, 180, 135, 90, 45, 0, 315
  };
  for (const auto &theta_deg : thetas_deg) {
    double x = r*std::cos(theta_deg*M_PI/180) + xc;
    double y = r*std::sin(theta_deg*M_PI/180) + yc;
    double yaw = (theta_deg-90)*M_PI/180;
    exploration_waypoints_.push_back(cg::planning::create_pose2d(x, y, yaw));
  }
  // Final waypoints for backing into the corner and driving forward
  exploration_waypoints_.push_back(cg::planning::create_pose2d(
    w-min_dist_from_map_boundary_, min_dist_from_map_boundary_, 135*M_PI/180
  ));
  exploration_waypoints_.push_back(cg::planning::create_pose2d(
    2.0/3.0*w, 1.0/3.0*h, 135*M_PI/180
  ));
}

cg_msgs::msg::Pose2D ExplorationPlanner::getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map) {

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
