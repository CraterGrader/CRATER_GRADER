#ifndef PLANNING__TRANSPORT_PLANNER_HPP
#define PLANNING__TRANSPORT_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/site_map.hpp>
#include "ortools/linear_solver/linear_solver.h"
namespace cg {
namespace planning {

class TransportPlanner : public GoalPlanner {

public:
  TransportPlanner() {};
  cg_msgs::msg::Pose2D getGoalPose(
    const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::SiteMap &map);
  float basicExample(); // Example from ortools, should always return optimal value of 4: https://developers.google.com/optimization/introduction/cpp#complete-program
private: 
  // tuple<std::vector<cg::mapping::indexPoint>, std::vector<cg::mapping::indexPoint> > = gen_nodes_from_height_map(std::vector<float> map);
 
  // distance_matrix  setup_distance_matrix(src_nodes, sink_nodes)
 
  // src_array, sink_array = setup_height_arrays(src_nodes, sink_nodes)
 
  // policy solve_emd(sink_array, src_array, distance_matrix)
 
  // transport_list = get_transport_list(policy, src_nodes, sink_nodes)
 
  // nearest_transport, transport_idx = find_nearest_transport(transport_list, agent.pose, visited_transport_indices)

};

} // namespace planning
} // namespace cg

#endif // PLANNING__TRANSPORT_PLANNER_HPP
