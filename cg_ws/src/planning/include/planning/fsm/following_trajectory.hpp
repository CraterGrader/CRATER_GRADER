#ifndef PLANNING__FOLLOWING_TRAJECTORY_HPP
#define PLANNING__FOLLOWING_TRAJECTORY_HPP

#include <planning/fsm/fsm.hpp>
#include <cg_msgs/msg/pose2_d.hpp>
#include <planning/common.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class FollowingTrajectory : public FSM {

public:
  /**
   * @brief 
   * 
   * @param current_agent_pose 
   * @param current_goal_pose 
   * @param thresh_pos 
   * @param thresh_head 
   * @return true If goal pose reached
   * @return false If goal pose not reached
   */
  bool runState(const cg_msgs::msg::Pose2D &current_agent_pose, const cg_msgs::msg::Pose2D &current_goal_pose, const float thresh_pos, const double thresh_head); // Main function to run current state; optionally modifies signal and state for transition


}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__FOLLOWING_TRAJECTORY_HPP
