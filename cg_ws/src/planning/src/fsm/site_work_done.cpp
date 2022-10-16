#include <planning/fsm/site_work_done.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void SiteWorkDone::runState() {
  std::cout << "SITE_WORK_DONE" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::NO;
  curr_state_ = State::MAP_EXPLORED;
}

} // planning namespace
} // cg namespace
