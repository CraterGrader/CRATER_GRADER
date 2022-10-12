#ifndef PLANNING__GOAL_PLANNER_HPP
#define PLANNING__GOAL_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/map.hpp>

namespace cg {
namespace planning {

class GoalPlanner {

public:
  virtual std::vector<cg_msgs::msg::Pose2D> getGoalPose(
    const cg_msgs::msg::Pose2D& agent_pose, const cg::mapping::Map<float>& map) = 0;
};

} // namespace planning
} // namespace cg

#endif // PLANNING__GOAL_PLANNER_HPP
