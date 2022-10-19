#include <planning/fsm/get_worksystem_trajectory.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

  void GetWorksystemTrajectory::runStateMultiGoal(
    cg::planning::KinematicPlanner &kinematic_planner_, 
    const std::vector<cg_msgs::msg::Pose2D> &current_goal_poses_,
    std::vector<std::vector<cg_msgs::msg::Pose2D>> &current_trajectories_,
    const cg_msgs::msg::Pose2D &current_agent_pose_,
    cg::mapping::Map<float> &current_height_map_) {

  // Reset current_trajectories, using all current_goal_poses_
  current_trajectories_.clear();

  cg_msgs::msg::Pose2D start_pose = current_agent_pose_;
  for (cg_msgs::msg::Pose2D goal_pose: current_goal_poses_) {

    std::vector<cg_msgs::msg::Pose2D> path;
    kinematic_planner_.generatePath(path, start_pose, goal_pose, current_height_map_);
    current_trajectories_.push_back(path);
    start_pose = path.back();

  }

  std::cout << "GET_WORKSYSTEM_TRAJECTORY" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::FOLLOW_TRAJECTORY;
  curr_state_ = State::FOLLOWING_TRAJECTORY;
}

void GetWorksystemTrajectory::runState(const bool worksystem_enabled, bool &updated_trajectory, bool &calculated_trajectory)
{
  std::cout << "GET_WORKSYSTEM_TRAJECTORY" << std::endl;

  if (worksystem_enabled && updated_trajectory) {
    // Reset state flags for next trajectory iteration, if sent and enabled correctly
    updated_trajectory = false;
    calculated_trajectory = false;

    // Update shared current state and the precursing signal if trajectory was sent and worksystem is now enabled
    pre_signal_ = Signal::FOLLOW_TRAJECTORY;
    curr_state_ = State::FOLLOWING_TRAJECTORY;
  }
}

} // planning namespace
} // cg namespace
