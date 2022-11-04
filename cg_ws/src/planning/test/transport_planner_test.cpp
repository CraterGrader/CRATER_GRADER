#include <gtest/gtest.h>
#include <limits> // for infinity
#include <planning/transport_planner.hpp>

TEST(TransportPlannerTest, vector_map) {
  // Make height data
  std::vector<float> heightMapStandIn{
      0, 0, 0,   0,    0,    0,    0,   0, 0, 0, 0, 0,   0, 0,    0, 0,   0, 0,
      0, 0, 0,   1.0,  1.0,  1.0,  0,   0, 0, 0, 0, 1.0, 0, -2.0, 0, 1.0, 0, 0,
      0, 0, 1.0, -2.0, -4.0, -2.0, 1.0, 0, 0, 0, 0, 1.0, 0, -2.0, 0, 1.0, 0, 0,
      0, 0, 0,   1.0,  1.0,  1.0,  0,   0, 0, 0, 0, 0,   0, 0,    0, 0,   0, 0,
      0, 0, 0,   0,    0,    0,    0,   0, 0};

  std::vector<float> designTOPO{
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Create maps
  size_t map_height_cells = 9;
  size_t map_width_cells = 9;
  float resolution = 0.1;
  cg::mapping::Map<float> current_height_map(map_height_cells, map_width_cells,
                                             resolution, heightMapStandIn);
  cg::mapping::Map<float> design_height_map(map_height_cells, map_width_cells,
                                            resolution, designTOPO);
  float threshold_z =
      0.001; // symmetric height offset from design topo to make nodes for
  float thresh_max_assignment_distance =
      std::numeric_limits<float>::infinity(); // something large so that all
                                              // assignments are used

  // Solve transport problem
  cg::planning::TransportPlanner transport_planner;
  std::vector<int> seen_map(map_height_cells * map_width_cells, 1);
  float objective_value = transport_planner.planTransport(
      current_height_map, design_height_map, seen_map, threshold_z,
      thresh_max_assignment_distance);
  std::vector<cg::planning::TransportAssignment> transport_assignments =
      transport_planner.getTransportAssignments();

  // Check number of transport assignments
  size_t expected_assignments = 12;
  size_t actual_assignments = transport_assignments.size();
  EXPECT_EQ(expected_assignments, actual_assignments);

  // Check objective value
  float expected_obj = 1.86f;
  float actual_obj = objective_value;
  EXPECT_NEAR(expected_obj, actual_obj, 0.1);
}

TEST(TransportPlannerTest, toy_problem) {
  // Solve transport problem (nodes created manually inside method)
  cg::planning::TransportPlanner transport_planner;
  float val = transport_planner.solveToyProblem();

  // Check objective value
  float expected = 1.8f;
  float actual = val;
  EXPECT_NEAR(expected, actual, 0.01);
}
