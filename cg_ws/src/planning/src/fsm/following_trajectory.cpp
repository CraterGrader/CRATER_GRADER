#include <planning/fsm/following_trajectory.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

bool FollowingTrajectory::runState(const cg_msgs::msg::Pose2D &current_agent_pose, const cg_msgs::msg::Pose2D &current_goal_pose, const float thresh_pos, const double thresh_head) {
  std::cout << "FOLLOWING_TRAJECTORY" << std::endl;

  if (cg::planning::samePoseWithinThresh(current_agent_pose, current_goal_pose, thresh_pos, thresh_head)) {
    // Update shared current state and the precursing signal
    pre_signal_ = Signal::GOAL_REACHED;
    curr_state_ = State::GOALS_REMAINING;
    return true;
  }
  return false;
}

} // planning namespace
} // cg namespace
