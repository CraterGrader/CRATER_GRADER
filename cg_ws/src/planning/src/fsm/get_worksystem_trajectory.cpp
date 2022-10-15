#include <planning/fsm/get_worksystem_trajectory.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void GetWorksystemTrajectory::runState() {
  std::cout << "GET_WORKSYSTEM_TRAJECTORY" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::STOP;
  curr_state_ = State::STOPPED;
}

} // planning namespace
} // cg namespace
