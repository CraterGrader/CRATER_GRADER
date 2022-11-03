#include <planning/fsm/map_explored.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void MapExplored::runState() {
  std::cout << "MAP_EXPLORED" << std::endl;

  // Update shared current state and the precursing signal
  // pre_signal_ = Signal::YES;
  // curr_state_l0_ = StateL0::REPLAN_TRANSPORT;
  pre_signal_ = Signal::NO;
  curr_state_l0_ = StateL0::PLAN_EXPLORATION;
}

} // planning namespace
} // cg namespace
