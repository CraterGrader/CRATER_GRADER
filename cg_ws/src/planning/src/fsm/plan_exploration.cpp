#include <planning/fsm/plan_exploration.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void PlanExploration::runState() {
  std::cout << "PLAN_EXPLORATION" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::EXPLORATION_PLANNED;
  curr_state_ = State::GET_EXPLORATION_GOALS;
}

} // planning namespace
} // cg namespace
