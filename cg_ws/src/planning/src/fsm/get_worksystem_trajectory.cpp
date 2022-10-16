#include <planning/fsm/get_worksystem_trajectory.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void GetWorksystemTrajectory::runState() {
  std::cout << "GET_WORKSYSTEM_TRAJECTORY" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::FOLLOW_TRAJECTORY;
  curr_state_ = State::FOLLOWING_TRAJECTORY;
}

} // planning namespace
} // cg namespace
