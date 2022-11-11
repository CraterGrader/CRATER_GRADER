#include <planning/fsm/map_explored.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void MapExplored::runState(float current_map_coverage_ratio_, float map_coverage_threshold) {
  std::cout << "MAP_EXPLORED" << std::endl;

  // Update shared current state and the precursing signal
  if (explored_once_) {
    pre_signal_ = Signal::YES;
    curr_state_l0_ = StateL0::REPLAN_TRANSPORT;
    curr_state_l1_ = StateL1::TRANSPORT;
  }
  else {
    explored_once_ = true;
    pre_signal_ = Signal::NO;
    curr_state_l0_ = StateL0::PLAN_EXPLORATION;
    curr_state_l1_ = StateL1::EXPLORATION;
  }
  // pre_signal_ = Signal::NO; // DEBUG
  // curr_state_l0_ = StateL0::PLAN_EXPLORATION; // DEBUG
  // curr_state_l1_ = StateL1::EXPLORATION; // DEBUG
}

} // planning namespace
} // cg namespace
