#include <gtest/gtest.h>
// Finite state machine and states
#include <planning/fsm/fsm.hpp>
#include <planning/fsm/update_map.hpp>
#include <planning/fsm/site_work_done.hpp>
#include <planning/fsm/map_explored.hpp>
#include <iostream> // DEBUG


TEST(FSMTest, update_test)
{
  // Create Finite State Machine
  cg::planning::FSM fsm;

  // Initialize states
  cg::planning::UpdateMap update_map;
  cg::planning::SiteWorkDone site_work_done;
  cg::planning::MapExplored map_explored;

  // Run machine
  std::cout << "~~~~~~~ Running machine..." << std::endl;
  std::cout << "~~~~~~~ Machine iteration" << std::endl;
  for (int i=0; i < 5; ++i) {
    std::cout << "    Pre-Signal: " << fsm.preSignalToString() << std::endl;
    std::cout << "         State: " << fsm.currStateToString() << std::endl;
    switch (fsm.getCurrState()) {
    case cg::planning::FSM::State::UPDATE_MAP:
      update_map.runState();
      break;
    case cg::planning::FSM::State::SITE_WORK_DONE:
      site_work_done.runState();
      break;
    case cg::planning::FSM::State::MAP_EXPLORED:
      map_explored.runState();
      break;
    default:
      break;
    }
  }
  std::cout << "~~~~~~~ Machine done!" << std::endl;

  float expected = 0.0f;
  float actual = 0.0f;
  float absolute_range = 0.0f;
  EXPECT_NEAR(expected, actual, absolute_range);
}

TEST(FSMTest, init_manual_test)
{ 
  // Create Finite State Machine
  cg::planning::FSM::State start_state = cg::planning::FSM::State::MAP_EXPLORED;
  cg::planning::FSM::Signal start_signal = cg::planning::FSM::Signal::MAP_UPDATED;
  cg::planning::FSM fsm(start_state, start_signal);

  // Check for default start state and signal
  EXPECT_EQ(fsm.getCurrState(), start_state);
  EXPECT_EQ(fsm.getPreSignal(), start_signal);
}

TEST(FSMTest, init_default_test)
{
  // Create Finite State Machine
  cg::planning::FSM fsm;

  // Check for default start state and signal
  EXPECT_EQ(fsm.getCurrState(), cg::planning::FSM::State::UPDATE_MAP);
  EXPECT_EQ(fsm.getPreSignal(), cg::planning::FSM::Signal::START);
}
