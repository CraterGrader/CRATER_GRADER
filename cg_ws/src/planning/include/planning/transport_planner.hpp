#ifndef PLANNING__TRANSPORT_PLANNER_HPP
#define PLANNING__TRANSPORT_PLANNER_HPP

#include <planning/common.hpp>
#include <planning/goal_planner.hpp>
#include <mapping/map.hpp>
#include "ortools/linear_solver/linear_solver.h"
#include <cmath> // for math operations, sqrt & floor 
#include <limits> // used for distance

namespace cg {
namespace planning {

struct TransportNode {
  // a struct which simply defines a points position in continous 3-space
    float x;
    float y;
    float height;
};

struct TransportAssignment {
  TransportNode source_node;
  TransportNode sink_node;
  float transport_volume;
};

class TransportPlanner : public GoalPlanner {

public:
  // Constructor()
  TransportPlanner() {};

  // Computations()
  float planTransport(const cg::mapping::Map<float> &current_height_map, const cg::mapping::Map<float> &design_height_map, const float threshold_z);
  std::vector<cg_msgs::msg::Pose2D> getGoalPose(const cg_msgs::msg::Pose2D &agent_pose, const cg::mapping::Map<float> &map);

  float solveToyProblem(); // For implementation verification purposes only

  // Helpers()
  size_t ij_to_index(size_t x, size_t y, size_t width) const;
  void init_nodes(std::vector<TransportNode> &source_nodes, std::vector<TransportNode> &sink_nodes, float &vol_source, float &vol_sink, const cg::mapping::Map<float> &current_height_map, const cg::mapping::Map<float> &design_height_map, const float threshold_z);
  void calculate_distances(std::vector<float> &distances_between_nodes, const std::vector<TransportNode> &source_nodes, const std::vector<TransportNode> &sink_nodes);
  float solveForTransportAssignments(std::vector<TransportAssignment> &new_transport_assignments, const std::vector<TransportNode> &source_nodes, const std::vector<TransportNode> &sink_nodes, const std::vector<float> &distances_between_nodes, const float vol_source, const float vol_sink, bool verbose);

  // Getters()
  std::vector<TransportAssignment> getTransportAssignments() const {return transport_assignments_;};

private: 
  // Attributes
  std::vector<TransportAssignment> transport_assignments_; // Assignments for transporting volume from a source to a sink (i.e. the non-zero transports)
  std::vector<bool> unvisited_assignments_; // each index corresponds to a TransportAssignment in transport_assignments. true = unvisited, false = visited

};

} // namespace planning
} // namespace cg

#endif // PLANNING__TRANSPORT_PLANNER_HPP
