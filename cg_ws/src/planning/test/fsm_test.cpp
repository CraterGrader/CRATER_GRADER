#include <gtest/gtest.h>
// Finite state machine and states
#include <planning/fsm/fsm.hpp>

TEST(FSMTest, init_manual_test)
{ 
  // Create Finite State Machine
  cg::planning::FSM::Phase start_phase = cg::planning::FSM::Phase::TRANSPORT;
  cg::planning::FSM::State start_state = cg::planning::FSM::State::MAP_EXPLORED;
  cg::planning::FSM::Signal start_signal = cg::planning::FSM::Signal::MAP_UPDATED;
  cg::planning::FSM fsm(start_phase, start_state, start_signal);

  // Check for default start state and signal
  EXPECT_EQ(fsm.getCurrPhase(), start_phase);
  EXPECT_EQ(fsm.getCurrState(), start_state);
  EXPECT_EQ(fsm.getPreSignal(), start_signal);
}

TEST(FSMTest, init_default_test)
{
  // Create Finite State Machine
  cg::planning::FSM fsm;

  // Check for default start state and signal
  EXPECT_EQ(fsm.getCurrPhase(), cg::planning::FSM::Phase::BEGINNING);
  EXPECT_EQ(fsm.getCurrState(), cg::planning::FSM::State::READY);
  EXPECT_EQ(fsm.getPreSignal(), cg::planning::FSM::Signal::START);
}
