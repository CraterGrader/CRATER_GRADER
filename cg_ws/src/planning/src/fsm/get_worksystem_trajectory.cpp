#include <planning/fsm/get_worksystem_trajectory.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

  void GetWorksystemTrajectory::runState(
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
  pre_signal_ = Signal::STOP;
  curr_state_ = State::STOPPED;
}

} // planning namespace
} // cg namespace
