#include <planning/fsm/get_transport_goals.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

  void GetTransportGoals::runState(std::vector<cg_msgs::msg::Pose2D> &current_goal_poses, std::vector<cg_msgs::msg::Pose2D> &state_l1_goal_poses, cg::planning::TransportPlanner &transport_planner, const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map)
  {
    std::cout << "GET_TRANSPORT_GOALS" << std::endl;

    std::vector<cg_msgs::msg::Pose2D> goalPoses = transport_planner.getGoalPose(agent_pose, map);
    while (goalPoses.size() > 0) {
      state_l1_goal_poses.insert(state_l1_goal_poses.end(), goalPoses.begin(), goalPoses.end());
      goalPoses = transport_planner.getGoalPose(agent_pose, map);
    }

    current_goal_poses = transport_planner.getGoalPose(agent_pose, map);

    // Update shared current state and the precursing signal
    pre_signal_ = Signal::DRIVE;
    curr_state_ = State::GOALS_REMAINING;
  }

} // planning namespace
} // cg namespace
