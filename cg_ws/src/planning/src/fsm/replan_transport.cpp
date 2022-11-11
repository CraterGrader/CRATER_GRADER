#include <planning/fsm/replan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void ReplanTransport::runState() {
  std::cout << "REPLAN_TRANSPORT" << std::endl;

  // For now, only replan the first time through
  // If causing computational bottleneck later on
  // compare to map used during last transport planning 

  // Update shared current state and the precursing signal
  if (!transport_planned) {
    pre_signal_ = Signal::YES;
    curr_state_l0_ = StateL0::PLAN_TRANSPORT;
    transport_planned = true;
    return;
  }

  pre_signal_ = Signal::NO;
  curr_state_l0_ = StateL0::GET_TRANSPORT_GOALS;
}

} // planning namespace
} // cg namespace
