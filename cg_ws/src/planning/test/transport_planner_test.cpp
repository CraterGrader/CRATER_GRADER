#include <gtest/gtest.h>
#include <planning/transport_planner.hpp>
// #include <planning/ceres_helloworld.hpp>

TEST(TransportPlannerTest, helloworld){
  float expected = 0.0f;
  float actual = 0.0f;
  float absolute_range = 0.0f;
  EXPECT_NEAR(expected, actual, absolute_range);
}

TEST(TransportPlannerTest, ortools_helloworld){
  cg::planning::TransportPlanner transport_planner;
  // Example from ortools, should always return optimal value of 4: https://developers.google.com/optimization/introduction/cpp#complete-program
  float val = transport_planner.basicExample();

  float expected = 4.0f;
  float actual = val;
  EXPECT_EQ(expected, actual);
}

TEST(TransportPlannerTest, ortools_EMD_baby){
  cg::planning::TransportPlanner transport_planner;
  // Example from ortools, should always return optimal value of 4: https://developers.google.com/optimization/introduction/cpp#complete-program
  float val = transport_planner.solveEMDtoy();

  float expected = 1.8f;
  float actual = val;
  EXPECT_NEAR(expected, actual,10.0);
}

// colcon test --packages-select planning --event-handlers console_cohesion+
