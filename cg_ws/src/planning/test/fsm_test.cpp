#include <gtest/gtest.h>
// Finite state machine and states
#include <planning/fsm/fsm.hpp>
#include <planning/fsm/ready.hpp>
#include <planning/fsm/update_map.hpp>
#include <planning/fsm/site_work_done.hpp>
#include <planning/fsm/map_explored.hpp>
#include <planning/fsm/replan_transport.hpp>
#include <planning/fsm/plan_transport.hpp>
#include <planning/fsm/get_transport_goals.hpp>
#include <planning/fsm/plan_exploration.hpp>
#include <planning/fsm/get_exploration_goals.hpp>
#include <planning/fsm/goals_remaining.hpp>
#include <planning/fsm/get_worksystem_trajectory.hpp>
#include <planning/fsm/stopped.hpp>
// #include <iostream> // DEBUG


// TEST(FSMTest, update_test)
// {
//   // Create Finite State Machine
//   cg::planning::FSM fsm;

//   // Initialize states
//   cg::planning::Ready ready;
//   cg::planning::UpdateMap update_map;
//   cg::planning::SiteWorkDone site_work_done;
//   cg::planning::MapExplored map_explored;
//   cg::planning::ReplanTransport replan_transport;
//   cg::planning::PlanTransport plan_transport;
//   cg::planning::GetTransportGoals get_transport_goals;
//   cg::planning::PlanExploration plan_exploration;
//   cg::planning::GetExplorationGoals get_exploration_goals;
//   cg::planning::GoalsRemaining goals_remaining;
//   cg::planning::GetWorksystemTrajectory get_worksystem_trajectory;
//   cg::planning::Stopped stopped;

//   // Run machine
//   std::cout << "~~~~~~~ Running machine..." << std::endl;
//   std::cout << "~~~~~~~ Machine iteration" << std::endl;
//   for (int i=0; i < 15; ++i) {
//     std::cout << "    Pre-Signal: " << fsm.preSignalToString() << std::endl;
//     std::cout << "         State: " << fsm.currStateToString() << std::endl;
//     switch (fsm.getCurrState()) {
//       case cg::planning::FSM::State::READY:
//         ready.runState();
//         break;
//       case cg::planning::FSM::State::UPDATE_MAP:
//         update_map.runState();
//         break;
//       case cg::planning::FSM::State::SITE_WORK_DONE:
//         site_work_done.runState();
//         break;
//       case cg::planning::FSM::State::MAP_EXPLORED:
//         map_explored.runState();
//         break;
//       case cg::planning::FSM::State::REPLAN_TRANSPORT:
//         replan_transport.runState();
//         break;
//       case cg::planning::FSM::State::PLAN_TRANSPORT:
//         plan_transport.runState();
//         break;
//       case cg::planning::FSM::State::GET_TRANSPORT_GOALS:
//         get_transport_goals.runState();
//         break;
//       case cg::planning::FSM::State::PLAN_EXPLORATION:
//         plan_exploration.runState();
//         break;
//       case cg::planning::FSM::State::GET_EXPLORATION_GOALS:
//         get_exploration_goals.runState();
//         break;
//       case cg::planning::FSM::State::GOALS_REMAINING:
//         goals_remaining.runState();
//         break;
//       case cg::planning::FSM::State::GET_WORKSYSTEM_TRAJECTORY:
//         get_worksystem_trajectory.runState();
//         break;
//       case cg::planning::FSM::State::STOPPED:
//         stopped.runState();
//         break;
//       default:
//         std::cout << "~ ~ ~ ~ ! Invalid State !" << std::endl;
//         break;
//     }
//   }
//   std::cout << "~~~~~~~ Machine done!" << std::endl;

//   float expected = 0.0f;
//   float actual = 0.0f;
//   float absolute_range = 0.0f;
//   EXPECT_NEAR(expected, actual, absolute_range);
// }

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
  EXPECT_EQ(fsm.getCurrState(), cg::planning::FSM::State::READY);
  EXPECT_EQ(fsm.getPreSignal(), cg::planning::FSM::Signal::START);
}
