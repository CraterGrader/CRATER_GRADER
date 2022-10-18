#ifndef PLANNING__GET_WORKSYSTEM_TRAJECTORY_HPP
#define PLANNING__GET_WORKSYSTEM_TRAJECTORY_HPP

#include <planning/fsm/fsm.hpp>
#include <planning/kinematic_planner.hpp>


namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class GetWorksystemTrajectory : public FSM {

public:
  void runStateMultiGoal(
    cg::planning::KinematicPlanner &kinematic_planner_, 
    const std::vector<cg_msgs::msg::Pose2D> &current_goal_poses_,
    std::vector<std::vector<cg_msgs::msg::Pose2D>> &current_trajectories_,
    const cg_msgs::msg::Pose2D &current_agent_pose_,
    cg::mapping::Map<float> &current_height_map_); // Main function to run current state; optionally modifies signal and state for transition
  void runState(const bool worksystem_enabled, bool &updated_trajectory, bool &calculated_trajectory); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__GET_WORKSYSTEM_TRAJECTORY_HPP
