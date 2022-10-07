#ifndef PLANNING__TRANSPORT_PLANNER_HPP
#define PLANNING__TRANSPORT_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/site_map.hpp>
#include "ortools/linear_solver/linear_solver.h"
#include <cmath> // for math operations, sqrt & floor 

namespace cg {
namespace planning {

struct TransportNode {
  // a struct which simply defines a points position in continous 3-space
    float x;
    float y;
    float height;
};

class TransportPlanner : public GoalPlanner {

public:
  TransportPlanner() {};
  cg_msgs::msg::Pose2D getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map);
  float basicExample(); // Example from ortools, should always return optimal value of 4: https://developers.google.com/optimization/introduction/cpp#complete-program

  size_t xy_to_index(size_t x, size_t y, size_t width);

  float solveEMDtoy(); // prototype of EMD solve
  float solveEMDtoyLoop();
  float solveEMDhardMap(); // EMD Solve hard coded map


private: 

  

};

} // namespace planning
} // namespace cg

#endif // PLANNING__TRANSPORT_PLANNER_HPP
