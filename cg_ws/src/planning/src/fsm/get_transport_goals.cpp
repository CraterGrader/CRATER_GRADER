#include <planning/fsm/get_transport_goals.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

  void GetTransportGoals::runState(std::vector<cg_msgs::msg::Pose2D> &current_goal_poses, cg::planning::TransportPlanner &transport_planner, const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map)
  {
    std::cout << "GET_TRANSPORT_GOALS" << std::endl;
    current_goal_poses = transport_planner.getGoalPose(agent_pose, map);

    // Update shared current state and the precursing signal
    pre_signal_ = Signal::DRIVE;
    curr_state_ = State::GOALS_REMAINING;
  }

} // planning namespace
} // cg namespace
