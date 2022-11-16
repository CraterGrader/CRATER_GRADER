#ifndef PLANNING__FSM_HPP
#define PLANNING__FSM_HPP

#include <string> // for converting enums to string

namespace cg {
namespace planning {

class FSM {

public:
  enum class StateL0 {
    READY,
    UPDATE_MAP,
    SITE_WORK_DONE,
    MAP_EXPLORED,
    REPLAN_TRANSPORT,
    PLAN_TRANSPORT,
    GET_TRANSPORT_GOALS,
    PLAN_EXPLORATION,
    GET_EXPLORATION_GOALS,
    GOALS_REMAINING,
    GET_WORKSYSTEM_TRAJECTORY,
    FOLLOWING_TRAJECTORY,
    END_MISSION,
    STOPPED
  };

  enum class StateL1 {
    EXPLORATION,
    TRANSPORT
  };

  enum class Signal {
    START,
    STOP,
    YES,
    NO,
    MAP_UPDATED,
    TRANSPORT_PLANNED,
    EXPLORATION_PLANNED,
    DRIVE,
    FOLLOW_TRAJECTORY,
    GOAL_REACHED,
    REPLAN
  };

  // Constructors()
  FSM(){};
  FSM(StateL1 start_state_l1, StateL0 start_state_l0, Signal start_signal);
  
  // Destructor(), for resetting static variables
  ~FSM();

  // Getters()
  StateL1 getCurrStateL1() const { return curr_state_l1_; }
  StateL0 getCurrStateL0() const { return curr_state_l0_; }
  Signal getPreSignal() const { return pre_signal_; }

  // Setters()
  void setCurrStateL1(StateL1 curr_state_l1) { curr_state_l1_ = curr_state_l1; }
  void setCurrStateL0(StateL0 curr_state_l0) { curr_state_l0_ = curr_state_l0; }
  void setPreSignal(Signal pre_signal) { pre_signal_ = pre_signal; }

  // Helpers
  std::string currStateL1ToString();
  std::string currStateL0ToString();
  std::string preSignalToString();

protected: // "Shared private" variables
  static StateL1 curr_state_l1_; // Current StateL1 that FSM is in
  static StateL0 curr_state_l0_; // Current state that should run
  static Signal pre_signal_; // Precursing signal that led to the current state

private:
  /******************************/
  /**
   * These default...() methods should only be used by the FSM class!
   * - Used to initialize the static state and signal
   * - May need to update the init_default_test if these defaults change
   */
  static StateL1 defaultStartStateL1() { return StateL1::EXPLORATION; }
  static StateL0 defaultStartStateL0() { return StateL0::READY; }
  static Signal defaultStartSignal() { return Signal::START; }
  /******************************/

}; // class FSM

} // namespace planning
} // namespace cg

#endif // PLANNING__FSM_HPP
