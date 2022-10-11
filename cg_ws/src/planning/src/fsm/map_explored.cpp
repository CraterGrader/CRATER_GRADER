#include <planning/fsm/map_explored.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void MapExplored::runState() {
  std::cout << "MAP_EXPLORED" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::YES;
  curr_state_ = State::REPLAN_TRANSPORT;
}

} // planning namespace
} // cg namespace
