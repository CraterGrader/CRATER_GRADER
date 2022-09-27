#ifndef GOAL_PLANNER_HPP
#define GOAL_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/site_map.hpp>

namespace cg {
namespace planning {

class GoalPlanner {

public:
  virtual Pose getGoalPose(const Pose& agent_pose, const cg::mapping::SiteMap& map) = 0;

};

} // namespace planning
} // namespace cg

#endif // GOAL_PLANNER_HPP