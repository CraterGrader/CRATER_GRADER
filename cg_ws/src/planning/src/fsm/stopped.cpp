#include <iostream> // DEBUG
#include <planning/fsm/stopped.hpp>

namespace cg {
namespace planning {

void Stopped::runState() {
  std::cout << "STOPPED" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::STOP;
  curr_state_l0_ = StateL0::STOPPED;
}

} // namespace planning
} // namespace cg
