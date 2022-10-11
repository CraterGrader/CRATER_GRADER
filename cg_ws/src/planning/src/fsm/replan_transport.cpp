#include <planning/fsm/replan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void ReplanTransport::runState() {
  std::cout << "REPLAN_TRANSPORT" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::YES;
  curr_state_ = State::PLAN_TRANSPORT;
}

} // planning namespace
} // cg namespace
