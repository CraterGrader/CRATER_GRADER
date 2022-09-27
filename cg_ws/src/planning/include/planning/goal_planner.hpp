#ifndef PLANNING__GOAL_PLANNER_HPP
#define PLANNING__GOAL_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/site_map.hpp>

namespace cg {
namespace planning {

class GoalPlanner {

public:
  virtual Pose2D getGoalPose(const Pose2D& agent_pose, const cg::mapping::SiteMap& map) = 0;

};

} // namespace planning
} // namespace cg

#endif // PLANNING__GOAL_PLANNER_HPP
