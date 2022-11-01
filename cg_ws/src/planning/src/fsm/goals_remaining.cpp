#include <planning/fsm/goals_remaining.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void GoalsRemaining::runState(std::vector<cg_msgs::msg::Pose2D> &current_goal_poses, cg_msgs::msg::Pose2D &current_goal_pose) {
  std::cout << "GOALS_REMAINING" << std::endl;

  // Update shared current state and the precursing signal
  if (current_goal_poses.size() > 0) {
    // Update the current goal pose
    current_goal_pose = current_goal_poses[0];
    // Remove that goal pose from the list of poses
    current_goal_poses.erase(current_goal_poses.begin());

    // Change state to generate trajectory
    pre_signal_ = Signal::YES;
    curr_state_ = State::GET_WORKSYSTEM_TRAJECTORY;
    return;
  }

  // Otherwise no more goals are left so go back to checking the map
  pre_signal_ = Signal::NO;
  curr_state_ = State::UPDATE_MAP;
}

} // planning namespace
} // cg namespace
