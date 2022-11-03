#include <planning/fsm/update_map.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void UpdateMap::runState(const bool map_updated) {
  std::cout << "UPDATE_MAP" << std::endl;

  // Don't move to next state if map is not updated
  if (!map_updated) {
    return;
  }
  
  // Update shared current state and the precursing signal
  pre_signal_ = Signal::MAP_UPDATED;
  curr_state_l0_ = StateL0::SITE_WORK_DONE;

}

} // planning namespace
} // cg namespace
