#include <planning/fsm/site_work_done.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void SiteWorkDone::runState(const cg::mapping::Map<float> &current_height_map, const cg::mapping::Map<float> &design_height_map, float transport_threshold_z) {
  std::cout << "SITE_WORK_DONE" << std::endl;

  bool within_thresh = cg::mapping::mapSimilarityWithinThreshold(
    current_height_map.getCellData(), 
    design_height_map.getCellData(), 
    transport_threshold_z);

  // Update shared current state and the precursing signal
  if (within_thresh) {
    pre_signal_ = Signal::YES;
    curr_state_ = State::END_MISSION;
  } else {
  pre_signal_ = Signal::NO;
  curr_state_ = State::MAP_EXPLORED;

  }

}

} // planning namespace
} // cg namespace
