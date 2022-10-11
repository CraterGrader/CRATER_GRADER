#include <planning/fsm/fsm.hpp>

namespace cg {
namespace planning {

// Initialize default static variables
FSM::State FSM::curr_state_ = FSM::defaultStartState();
FSM::Signal FSM::pre_signal_ = FSM::defaultStartSignal();

FSM::FSM(State start_state, Signal start_signal) {
  curr_state_ = start_state;
  pre_signal_ = start_signal;
}

FSM::~FSM(){
  // Reset to default state when being destroyed so new FSM objects start with defaults (e.g. for tests)
  curr_state_ = FSM::defaultStartState();
  pre_signal_ = FSM::defaultStartSignal();
}

std::string FSM::currStateToString() {
  switch(curr_state_) {
    case State::UPDATE_MAP:
      return "UPDATE_MAP";
    case State::SITE_WORK_DONE:
      return "SITE_WORK_DONE";
    case State::MAP_EXPLORED:
      return "MAP_EXPLORED";
    case State::REPLAN_TRANSPORT:
      return "REPLAN_TRANSPORT";
    default:
      return "Invalid State!";
  }
}
std::string FSM::preSignalToString()
{
  switch(pre_signal_) {
    case Signal::START:
      return "START";
    case Signal::STOP:
      return "STOP";
    case Signal::YES:
      return "YES";
    case Signal::NO:
      return "NO";
    case Signal::MAP_UPDATED:
      return "MAP_UPDATED";
    default:
      return "Invalid Signal!";
  }
}

} // planning namespace
} // cg namespace
