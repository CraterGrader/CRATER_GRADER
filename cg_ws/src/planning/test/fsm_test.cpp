#include <gtest/gtest.h>
// Finite state machine and states
#include <planning/fsm/fsm.hpp>

TEST(FSMTest, init_manual_test)
{ 
  // Create Finite State Machine
  cg::planning::FSM::StateL1 start_state_l1 = cg::planning::FSM::StateL1::TRANSPORT;
  cg::planning::FSM::State start_state = cg::planning::FSM::State::MAP_EXPLORED;
  cg::planning::FSM::Signal start_signal = cg::planning::FSM::Signal::MAP_UPDATED;
  cg::planning::FSM fsm(start_state_l1, start_state, start_signal);

  // Check for default start state and signal
  EXPECT_EQ(fsm.getCurrStateL1(), start_state_l1);
  EXPECT_EQ(fsm.getCurrState(), start_state);
  EXPECT_EQ(fsm.getPreSignal(), start_signal);
}

TEST(FSMTest, init_default_test)
{
  // Create Finite State Machine
  cg::planning::FSM fsm;

  // Check for default start state and signal
  EXPECT_EQ(fsm.getCurrStateL1(), cg::planning::FSM::StateL1::BEGINNING);
  EXPECT_EQ(fsm.getCurrState(), cg::planning::FSM::State::READY);
  EXPECT_EQ(fsm.getPreSignal(), cg::planning::FSM::Signal::START);
}
