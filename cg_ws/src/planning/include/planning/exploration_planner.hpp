#ifndef PLANNING__EXPLORATION_PLANNER_HPP
#define PLANNING__EXPLORATION_PLANNER_HPP

#include <mapping/map.hpp>
#include <planning/common.hpp>
#include <planning/goal_planner.hpp>

namespace cg {
namespace planning {

class ExplorationPlanner : public GoalPlanner {

public:
  ExplorationPlanner(double min_dist_from_map_boundary)
      : min_dist_from_map_boundary_(min_dist_from_map_boundary){};
  bool planExploration(const cg::mapping::Map<float> &map);
  std::vector<cg_msgs::msg::Pose2D>
  getGoalPose(const cg_msgs::msg::Pose2D &agent_pose,
              const cg::mapping::Map<float> &map);

private:
  double min_dist_from_map_boundary_;
  std::vector<cg_msgs::msg::Pose2D> exploration_waypoints_;
};

} // namespace planning
} // namespace cg

#endif // PLANNING__EXPLORATION_PLANNER_HPP
