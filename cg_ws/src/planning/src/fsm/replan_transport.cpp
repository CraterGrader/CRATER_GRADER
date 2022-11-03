#include <planning/fsm/replan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void ReplanTransport::runState() {
  std::cout << "REPLAN_TRANSPORT" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::YES;
  curr_state_l0_ = StateL0::PLAN_TRANSPORT;
  // pre_signal_ = Signal::NO;
  // curr_state_l0_ = StateL0::GET_TRANSPORT_GOALS;
}

} // planning namespace
} // cg namespace
