#ifndef PLANNING__EXPLORATION_PLANNER_HPP
#define PLANNING__EXPLORATION_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/site_map.hpp>

namespace cg {
namespace planning {

class ExplorationPlanner : public GoalPlanner {

public:
  ExplorationPlanner() : curr_waypoint_idx_(0) {};
  cg_msgs::msg::Pose2D getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map);
  //
  bool planExploration(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map);

private:
  std::vector<cg_msgs::msg::Pose2D> exploration_waypoints_;
  int curr_waypoint_idx_;
};

} // namespace planning
} // namespace cg

#endif // PLANNING__EXPLORATION_PLANNER_HPP
