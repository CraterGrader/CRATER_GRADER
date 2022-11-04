#include <iostream> // DEBUG
#include <planning/fsm/replan_transport.hpp>

namespace cg {
namespace planning {

void ReplanTransport::runState() {
  std::cout << "REPLAN_TRANSPORT" << std::endl;

  // For now, always replan
  // If causing computational bottleneck later on
  // compare to map used during last transport planning

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::YES;
  curr_state_l0_ = StateL0::PLAN_TRANSPORT;
  // pre_signal_ = Signal::NO;
  // curr_state_l0_ = StateL0::GET_TRANSPORT_GOALS;
}

} // namespace planning
} // namespace cg
