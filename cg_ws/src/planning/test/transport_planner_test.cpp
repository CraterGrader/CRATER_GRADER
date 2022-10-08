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
  // Make height and 
  std::vector<float> heightMapStandIn{0,    0,    0,    0,    0,    0,    0,    0,    0, 
                                      0,    0,    0,    0,    0,    0,    0,    0,    0,
                                      0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,
                                      0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,
                                      0,    0,  1.0, -2.0, -4.0, -2.0,  1.0,    0,    0,
                                      0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,
                                      0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,
                                      0,    0,    0,    0,    0,    0,    0,    0,    0,
                                      0,    0,    0,    0,    0,    0,    0,    0,    0};

  std::vector<float> designTOPO{0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0,
                                  0,    0,    0,    0,    0,    0,    0,    0,    0, 
                                  0,    0,    0,    0,    0,    0,    0,    0,    0};

  size_t map_height_cells = 9;
  size_t map_width_cells = 9;
  float resolution = 0.1;
  cg::mapping::Map<float> current_height_map(map_height_cells, map_width_cells, resolution, heightMapStandIn);
  cg::mapping::Map<float> design_height_map(map_height_cells, map_width_cells, resolution, designTOPO);
  float threshold_z = 0.001; // symmetric height offset from design topo to make nodes for

  // Solve transport problem
  cg::planning::TransportPlanner transport_planner;
  float objective_value = transport_planner.solveEMDrealMap(current_height_map, design_height_map, threshold_z);
  std::vector<cg::planning::TransportAssignment> transport_assignments = transport_planner.getTransportAssignments();
  
  // Check number of transport assignments
  size_t expected_assignments = 12;
  size_t actual_assignments = transport_assignments.size();
  EXPECT_EQ(expected_assignments, actual_assignments);

  // Check objective value
  float expected_obj = 1.86f;
  float actual_obj = objective_value;
  EXPECT_NEAR(expected_obj, actual_obj, 0.1);
}
