#ifndef PLANNING__EXPLORATION_PLANNER_HPP
#define PLANNING__EXPLORATION_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/site_map.hpp>

namespace cg {
namespace planning {

class ExplorationPlanner : public GoalPlanner {

public:
  std::vector<cg_msgs::msg::Pose2D> getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map);
  void planExploration(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map);
  
};

} // namespace planning
} // namespace cg

#endif // PLANNING__EXPLORATION_PLANNER_HPP
