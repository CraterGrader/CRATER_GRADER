#include <planning/fsm/goals_remaining.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void GoalsRemaining::runState() {
  std::cout << "GOALS_REMAINING" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::YES;
  curr_state_ = State::GET_WORKSYSTEM_TRAJECTORY;
}

} // planning namespace
} // cg namespace
