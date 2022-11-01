#include <planning/fsm/stopped.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void Stopped::runState() {
  std::cout << "STOPPED" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::STOP;
  curr_state_ = State::STOPPED;
}

} // planning namespace
} // cg namespace
