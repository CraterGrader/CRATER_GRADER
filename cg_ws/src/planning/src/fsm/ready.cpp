#include <iostream> // DEBUG
#include <planning/fsm/ready.hpp>

namespace cg {
namespace planning {

void Ready::runState() {
  std::cout << "READY" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::START;
  curr_state_l0_ = StateL0::UPDATE_MAP;
}

} // namespace planning
} // namespace cg
