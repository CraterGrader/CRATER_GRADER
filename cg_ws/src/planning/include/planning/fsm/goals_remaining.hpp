#ifndef PLANNING__GOALS_REMAINING_HPP
#define PLANNING__GOALS_REMAINING_HPP

#include <planning/fsm/fsm.hpp>
#include <vector>
#include <cg_msgs/msg/pose2_d.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class GoalsRemaining : public FSM {

public:
  void runState(std::vector<cg_msgs::msg::Pose2D> &current_goal_poses, cg_msgs::msg::Pose2D &current_goal_pose); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__GOALS_REMAINING_HPP
