#include <planning/fsm/stopped.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void Stopped::runState() {
  std::cout << "STOPPED" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::STOP;
  curr_state_l0_ = StateL0::STOPPED;
}

} // planning namespace
} // cg namespace
