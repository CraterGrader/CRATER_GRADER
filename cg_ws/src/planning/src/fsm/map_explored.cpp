#include <iostream> // DEBUG
#include <planning/fsm/map_explored.hpp>

namespace cg {
namespace planning {

void MapExplored::runState(float current_map_coverage_ratio_,
                           float map_coverage_threshold) {
  std::cout << "MAP_EXPLORED" << std::endl;

  // Update shared current state and the precursing signal
  if (current_map_coverage_ratio_ >= map_coverage_threshold) {
    pre_signal_ = Signal::YES;
    curr_state_l0_ = StateL0::REPLAN_TRANSPORT;
    curr_state_l1_ = StateL1::TRANSPORT;
  } else {
    pre_signal_ = Signal::NO;
    curr_state_l0_ = StateL0::PLAN_EXPLORATION;
    curr_state_l1_ = StateL1::EXPLORATION;
  }
}

} // namespace planning
} // namespace cg
