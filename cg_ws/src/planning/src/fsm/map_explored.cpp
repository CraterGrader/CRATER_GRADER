#include <planning/fsm/map_explored.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void MapExplored::runState(float current_map_coverage_ratio_, float map_coverage_threshold) {
  std::cout << "MAP_EXPLORED" << std::endl;

  // Update shared current state and the precursing signal
  if (current_map_coverage_ratio_ >= map_coverage_threshold) {
    pre_signal_ = Signal::YES;
    curr_state_ = State::REPLAN_TRANSPORT;
  }
  else {
    pre_signal_ = Signal::NO;
    curr_state_ = State::PLAN_EXPLORATION;

  }
}

} // planning namespace
} // cg namespace
