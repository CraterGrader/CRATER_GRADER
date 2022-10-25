#include <planning/fsm/following_trajectory.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

bool FollowingTrajectory::runState(const cg_msgs::msg::Pose2D &current_agent_pose, const cg_msgs::msg::Pose2D &current_goal_pose, const float thresh_pos, const double thresh_head, const float thresh_euclidean_replan, const cg_msgs::msg::Trajectory &current_trajectory, const nav_msgs::msg::Odometry &global_robot_state, const cg_msgs::msg::Pose2D& global_robot_pose) {
  std::cout << "FOLLOWING_TRAJECTORY" << std::endl;

  // Check if re-plan is needed
  traj_idx_ = cg::planning::getClosestTrajIndex(current_trajectory, global_robot_state, traj_idx_);

  // euclidean_distance_to_trajectory_point_ = cg::planning::euclidean_distance(global_robot_pose.pt, current_trajectory.path[traj_idx_].pt);
  euclidean_distance_to_trajectory_point_ = 0.0;

  // std::cout << "      *    "  << global_robot_pose.pt.x << "       " << traj_idx_ << std::endl;

  // std::cout << "      *    "  << euclidean_distance_to_trajectory_point_ << "       " << traj_idx_ << std::endl;
  if (euclidean_distance_to_trajectory_point_ > thresh_euclidean_replan) {
    // Change state to re-calculate trajectory
    pre_signal_ = Signal::REPLAN;
    curr_state_ = State::GET_WORKSYSTEM_TRAJECTORY;
    traj_idx_ = 0; // Reset state trajectory index so next following state begins with zero
    return false;
  }

  if (cg::planning::samePoseWithinThresh(current_agent_pose, current_goal_pose, thresh_pos, thresh_head)) {
    // Update shared current state and the precursing signal
    pre_signal_ = Signal::GOAL_REACHED;
    curr_state_ = State::GOALS_REMAINING;
    traj_idx_ = 0; // Reset state trajectory index so next following state begins with zero
    return false;
  }
  return true;
}

} // planning namespace
} // cg namespace
