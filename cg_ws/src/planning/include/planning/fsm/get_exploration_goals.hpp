#ifndef PLANNING__GET_EXPLORATION_GOALS_HPP
#define PLANNING__GET_EXPLORATION_GOALS_HPP

#include <planning/fsm/fsm.hpp>
#include <planning/exploration_planner.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class GetExplorationGoals : public FSM {

public:
  void runState(std::vector<cg_msgs::msg::Pose2D> &current_goal_poses, std::vector<cg_msgs::msg::Pose2D> &phase_goal_poses, cg::planning::ExplorationPlanner &exploration_planner, const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__GET_EXPLORATION_GOALS_HPP
