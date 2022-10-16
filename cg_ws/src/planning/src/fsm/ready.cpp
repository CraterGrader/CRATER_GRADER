#include <planning/fsm/ready.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void Ready::runState() {
  std::cout << "READY" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::START;
  curr_state_ = State::UPDATE_MAP;
}

} // planning namespace
} // cg namespace
