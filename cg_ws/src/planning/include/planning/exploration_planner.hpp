#ifndef PLANNING__EXPLORATION_PLANNER_HPP
#define PLANNING__EXPLORATION_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/map.hpp>

namespace cg {
namespace planning {

class ExplorationPlanner : public GoalPlanner {

public:
  ExplorationPlanner() : curr_waypoint_idx_(0) {};
  cg_msgs::msg::Pose2D getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map);
  std::vector<cg_msgs::msg::Pose2D> planExploration(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map);
  bool isExplorationDone() const {return curr_waypoint_idx_ >= exploration_waypoints_.size();}

private:
  bool isGoalReached(const cg_msgs::msg::Pose2D &agent_pose, const cg_msgs::msg::Pose2D &goal_pose) const;
  std::vector<cg_msgs::msg::Pose2D> exploration_waypoints_;
  unsigned int curr_waypoint_idx_;

  static constexpr double xy_goal_thresh_ = 0.1;
  static constexpr double yaw_goal_thresh_ = 0.0872665; // 5 deg
  static constexpr double min_dist_from_map_boundary_ = 0.5;
};

} // namespace planning
} // namespace cg

#endif // PLANNING__EXPLORATION_PLANNER_HPP
