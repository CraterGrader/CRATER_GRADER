#include <planning/fsm/plan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void PlanTransport::runState() {
  std::cout << "PLAN_TRANSPORT" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::TRANSPORT_PLANNED;
  curr_state_ = State::GET_TRANSPORT_GOALS;
}

} // planning namespace
} // cg namespace
