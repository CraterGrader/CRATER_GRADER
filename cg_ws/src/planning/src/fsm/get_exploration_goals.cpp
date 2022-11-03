#include <planning/fsm/get_exploration_goals.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void GetExplorationGoals::runState(std::vector<cg_msgs::msg::Pose2D> &current_goal_poses, std::vector<cg_msgs::msg::Pose2D> &state_l1_goal_poses, cg::planning::ExplorationPlanner &exploration_planner, const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map){
  std::cout << "GET_EXPLORATION_GOALS" << std::endl;
  
  current_goal_poses = exploration_planner.getGoalPose(agent_pose, map);
  state_l1_goal_poses= current_goal_poses;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::DRIVE;
  curr_state_l0_ = StateL0::GOALS_REMAINING;
}

} // planning namespace
} // cg namespace
