#include <planning/fsm/plan_exploration.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void PlanExploration::runState(cg::planning::ExplorationPlanner &exploration_planner, const cg::mapping::Map<float> &map) {
  std::cout << "PLAN_EXPLORATION" << std::endl;

  exploration_planned_ = exploration_planner.planExploration(map);
  
  // Don't move to next state if planning was unsuccessful
  if (!exploration_planned_) {
    return;
  }

  // Update for shared current state and the precursing signal
  pre_signal_ = Signal::EXPLORATION_PLANNED;
  curr_state_ = State::GET_EXPLORATION_GOALS;
  exploration_planned_ = false; // Reset for next cycle
}

} // planning namespace
} // cg namespace
