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
   * @return true If robot should keep following
   * @return false If robot should stop following
   */
  bool runState(const cg_msgs::msg::Pose2D &current_agent_pose, const cg_msgs::msg::Pose2D &current_goal_pose, const float thresh_pos, const double thresh_head, const float thresh_euclidean_replan, const cg_msgs::msg::Trajectory &current_trajectory, const nav_msgs::msg::Odometry &global_robot_state, const cg_msgs::msg::Pose2D& global_robot_pose); // Main function to run current state; optionally modifies signal and state for transition

private:
  int traj_idx_ = 0; // used for tracking what index on trajectory is closest to current pose, monotonically increasing, reset when new traj given
  float euclidean_distance_to_trajectory_point_;

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__FOLLOWING_TRAJECTORY_HPP
