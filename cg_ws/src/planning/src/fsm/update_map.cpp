#include <planning/fsm/update_map.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void UpdateMap::runState() {
  std::cout << "UPDATE_MAP" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::MAP_UPDATED;
  curr_state_ = State::SITE_WORK_DONE;
}

} // planning namespace
} // cg namespace
