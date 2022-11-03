#include <planning/fsm/replan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void ReplanTransport::runState() {
  std::cout << "REPLAN_TRANSPORT" << std::endl;

  // For now, always replan
  // If causing computational bottleneck later on
  // compare to map used during last transport planning 

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::YES;
  curr_state_ = State::PLAN_TRANSPORT;
  // pre_signal_ = Signal::NO;
  // curr_state_ = State::GET_TRANSPORT_GOALS;
}

} // planning namespace
} // cg namespace
