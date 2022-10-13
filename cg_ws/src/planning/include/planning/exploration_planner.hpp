#ifndef PLANNING__EXPLORATION_PLANNER_HPP
#define PLANNING__EXPLORATION_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/map.hpp>

namespace cg {
namespace planning {

class ExplorationPlanner : public GoalPlanner {

public:
  ExplorationPlanner();
 
  std::vector<cg_msgs::msg::Pose2D> getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map);

private:
  static constexpr double min_dist_from_map_boundary_ = 0.5;
};

} // namespace planning
} // namespace cg

#endif // PLANNING__EXPLORATION_PLANNER_HPP
