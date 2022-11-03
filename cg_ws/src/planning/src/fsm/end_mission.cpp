#include <planning/fsm/end_mission.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

void EndMission::runState() {
  std::cout << "END_MISSION" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::STOP;
  curr_state_l0_ = StateL0::STOPPED;
}

} // planning namespace
} // cg namespace
