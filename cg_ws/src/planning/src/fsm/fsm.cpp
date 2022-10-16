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
    case State::READY:
      return "READY";
    case State::UPDATE_MAP:
      return "UPDATE_MAP";
    case State::SITE_WORK_DONE:
      return "SITE_WORK_DONE";
    case State::MAP_EXPLORED:
      return "MAP_EXPLORED";
    case State::REPLAN_TRANSPORT:
      return "REPLAN_TRANSPORT";
    case State::PLAN_TRANSPORT:
      return "PLAN_TRANSPORT";
    case State::GET_TRANSPORT_GOALS:
      return "GET_TRANSPORT_GOALS";
    case State::PLAN_EXPLORATION:
      return "PLAN_EXPLORATION";
    case State::GET_EXPLORATION_GOALS:
      return "GET_EXPLORATION_GOALS";
    case State::GOALS_REMAINING:
      return "GOALS_REMAINING";
    case State::GET_WORKSYSTEM_TRAJECTORY:
      return "GET_WORKSYSTEM_TRAJECTORY";
    case State::FOLLOWING_TRAJECTORY:
      return "FOLLOWING_TRAJECTORY";
    case State::STOPPED:
      return "STOPPED";
    default:
      return "State not recognized!";
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
    case Signal::TRANSPORT_PLANNED:
      return "TRANSPORT_PLANNED";
    case Signal::EXPLORATION_PLANNED:
      return "EXPLORATION_PLANNED";
    case Signal::DRIVE:
      return "DRIVE";
    case Signal::FOLLOW_TRAJECTORY:
      return "FOLLOW_TRAJECTORY";
    case Signal::GOAL_REACHED:
      return "GOAL_REACHED";
    default:
      return "Signal not recognized!";
  }
}

} // planning namespace
} // cg namespace
