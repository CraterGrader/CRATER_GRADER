#include <planning/fsm/plan_exploration.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void PlanExploration::runState(cg::planning::ExplorationPlanner &exploration_planner, const cg::mapping::Map<float> &map) {
  std::cout << "PLAN_EXPLORATION" << std::endl;

  exploration_planned_ = exploration_planner.planExploration(map);

  // Only update for shared current state and the precursing signal for valid plan
  if (exploration_planned_) {
    pre_signal_ = Signal::EXPLORATION_PLANNED;
    curr_state_ = State::GET_EXPLORATION_GOALS;
    exploration_planned_ = false; // Reset for next cycle
  }
}

} // planning namespace
} // cg namespace
