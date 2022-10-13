#include <planning/common.hpp>
#include <planning/exploration_planner.hpp>

namespace cg {
namespace planning {

/**
 * @brief // this function returns a vector of poses to drive during the exploration phase
 *
 * @param agent_pose // not used
 * @param map // used for parametric driving path
 * @return std::vector<cg_msgs::msg::Pose2D>
 */
std::vector<cg_msgs::msg::Pose2D> ExplorationPlanner::getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map) {
  (void)agent_pose; // agent_pose remains unused, legacy of interface 
  std::vector<cg_msgs::msg::Pose2D> exploration_waypoints;

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
    exploration_waypoints.push_back(cg::planning::create_pose2d(x, y, yaw));
  }
  // Final waypoints for backing into the corner and driving forward
  exploration_waypoints.push_back(cg::planning::create_pose2d(
    w-min_dist_from_map_boundary_, min_dist_from_map_boundary_, 135*M_PI/180
  ));
  exploration_waypoints.push_back(cg::planning::create_pose2d(
    2.0/3.0*w, 1.0/3.0*h, 135*M_PI/180
  ));

  return exploration_waypoints;
}

}  // namespace planning
}  // namespace cg
