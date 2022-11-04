#include <iostream> // DEBUG
#include <planning/fsm/end_mission.hpp>

namespace cg {
namespace planning {

void EndMission::runState() {
  std::cout << "END_MISSION" << std::endl;

  // Update shared current state and the precursing signal
  pre_signal_ = Signal::STOP;
  curr_state_l0_ = StateL0::STOPPED;
}

} // namespace planning
} // namespace cg
