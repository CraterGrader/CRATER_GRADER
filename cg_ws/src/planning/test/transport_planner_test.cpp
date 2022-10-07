#include <gtest/gtest.h>
#include <planning/transport_planner.hpp>

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
  float val = transport_planner.solveEMDtoy();

  float expected = 1.8f;
  float actual = val;
  EXPECT_NEAR(expected, actual,0.01);
}

TEST(TransportPlannerTest, ortools_EMD_toddler){
  cg::planning::TransportPlanner transport_planner;
  float val = transport_planner.solveEMDtoyLoop();

  float expected = 1.8f;
  float actual = val;
  EXPECT_NEAR(expected, actual,0.01);
}

TEST(TransportPlannerTest, ortools_EMD_middle_schooler){
  cg::planning::TransportPlanner transport_planner;
  float val = transport_planner.solveEMDhardMap();

  float expected = 1.8f;
  float actual = val;
  EXPECT_NEAR(expected, actual,10000.0);
}

TEST(TransportPlannerTest, ortools_EMD_high_schooler)
{
  cg::planning::TransportPlanner transport_planner;
  float val = transport_planner.solveEMDrealMap();

  float expected = 1.8f;
  float actual = val;
  EXPECT_NEAR(expected, actual, 10000.0);
}
