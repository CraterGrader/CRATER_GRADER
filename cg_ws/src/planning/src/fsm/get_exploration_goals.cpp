#include <planning/fsm/get_exploration_goals.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void GetExplorationGoals::runState() {
  std::cout << "GET_EXPLORATION_GOALS" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::DRIVE;
  curr_state_ = State::GOALS_REMAINING;
}

} // planning namespace
} // cg namespace
