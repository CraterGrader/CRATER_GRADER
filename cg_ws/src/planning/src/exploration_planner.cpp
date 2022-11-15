#include <planning/common.hpp>
#include <planning/exploration_planner.hpp>

namespace cg {
namespace planning {

bool ExplorationPlanner::planExploration(const cg::mapping::Map<float> &map) {

  // For now, have a fixed set of waypoints
  // Drive in a circle, then back into a corner and drive forward toward the center
  const double &h = map.getHeight()*map.getResolution();
  const double &w = map.getWidth()*map.getResolution();
  const double xc = w/2.0, yc = h/2.0;

  // Note: for now we assume driving a circle (constant radius)
  if (h != w){
    // Don't update waypoints for non-square map, since calculations will not align
    return false;
  }

  // Reset current waypoints to replan
  exploration_waypoints_.clear();
  // If the map is non-square, then technically we should be driving an ellipse
  double r = std::min(h,w)/2.0 - min_dist_from_map_boundary_;
  // Generate waypoints going clockwise around a circle
  std::vector<int> thetas_deg = {
    270, 225, 180, 135, 90, 45, 0//, 315
  };
  // Make sure that we close the circle
  thetas_deg.push_back(thetas_deg[0]);
  for (const auto &theta_deg : thetas_deg) {
    double x = r*std::cos(theta_deg*M_PI/180) + xc;
    double y = r*std::sin(theta_deg*M_PI/180) + yc;
    double yaw = (theta_deg-90)*M_PI/180;
    exploration_waypoints_.push_back(cg::planning::create_pose2d(x, y, yaw));
  }
  // Final waypoints for backing into the corner and driving forward
  exploration_waypoints_.push_back(cg::planning::create_pose2d(
    w/2.0, h/3.0, M_PI
  ));
  exploration_waypoints_.push_back(cg::planning::create_pose2d(
    0.8*w, h/5.0, 135*M_PI/180
  ));
  exploration_waypoints_.push_back(cg::planning::create_pose2d(
    2.0*w/3.0, h/3.0, 135*M_PI/180
  ));
  // exploration_waypoints_.push_back(cg::planning::create_pose2d(
  //   w-min_dist_from_map_boundary_, min_dist_from_map_boundary_, 135*M_PI/180
  // ));
  // exploration_waypoints_.push_back(cg::planning::create_pose2d(
  //   1.0/2.0*w - h/(2.0*std::sqrt(3)) + h/(3.0*std::sqrt(3)), 1.0/3.0*h, M_PI/3.0
  // ));

  return true;
}

/**
 * @brief // this function returns a vector of poses to drive during the exploration phase
 *
 * @param agent_pose // not used
 * @param map // not used
 * @return std::vector<cg_msgs::msg::Pose2D>
 */
std::vector<cg_msgs::msg::Pose2D> ExplorationPlanner::getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map) {
  (void)agent_pose; // agent_pose remains unused, legacy of interface
  (void)map; // map remains unused, legacy of interface
  // std::vector<cg_msgs::msg::Pose2D> exploration_waypoints;

  // // For now, have a fixed set of waypoints
  // // Drive in a circle, then back into a corner and drive forward toward the center
  // const double &h = map.getHeight()*map.getResolution();
  // const double &w = map.getWidth()*map.getResolution();
  // const double xc = w/2.0, yc = h/2.0;

  // // Note: for now we assume driving a circle (constant radius)
  // // If the map is non-square, then technically we should be driving an ellipse
  // double r = std::min(h,w)/2.0 - min_dist_from_map_boundary_;
  // // Generate waypoints going clockwise around a circle
  // std::vector<int> thetas_deg = {
  //   270, 225, 180, 135, 90, 45, 0, 315
  // };
  // // Make sure that we close the circle
  // thetas_deg.push_back(thetas_deg[0]);
  // for (const auto &theta_deg : thetas_deg) {
  //   double x = r*std::cos(theta_deg*M_PI/180) + xc;
  //   double y = r*std::sin(theta_deg*M_PI/180) + yc;
  //   double yaw = (theta_deg-90)*M_PI/180;
  //   exploration_waypoints.push_back(cg::planning::create_pose2d(x, y, yaw));
  // }
  // // Final waypoints for backing into the corner and driving forward
  // exploration_waypoints.push_back(cg::planning::create_pose2d(
  //   w-min_dist_from_map_boundary_, min_dist_from_map_boundary_, 135*M_PI/180
  // ));
  // exploration_waypoints.push_back(cg::planning::create_pose2d(
  //   2.0/3.0*w, 1.0/3.0*h, 135*M_PI/180
  // ));

  return exploration_waypoints_;
}

}  // namespace planning
}  // namespace cg
