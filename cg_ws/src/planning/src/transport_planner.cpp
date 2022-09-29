#include <planning/transport_planner.hpp>

namespace cg {
namespace planning {

// def gen_nodes_from_height_map(map):
//   # Convert map cells to nodes
//   src_nodes = []
//   sink_nodes = []
//   # Loop through height map and assign points to either source or sink
//   for idx in range(map.height * map.width):
//       # Get associated coordinates with these indices
//       x, y = map.idx_to_coords(idx)
//       # Positive height becomes a source; positive volume in +z
//       height = map.get_height(idx)
//       if height > 0.0:
//         src_nodes.append(Node(x, y, height))
//         continue
//       # Negative height becomes a sink; for solver the sinks also must have positive volume, defined as positive in -z
//       if height < 0.0:
//         sink_nodes.append(Node(x, y, -height))
//         continue
//   return src_nodes, sink_nodes


// def setup_distance_matrix(src_nodes, sink_nodes):
//   d = []
//   for src_node in src_nodes:
//     for sink_node in sink_nodes:
//       d.append(distance_function(
//           np.array([sink_node.x, sink_node.y]), np.array([src_node.x, src_node.y])))
//   d = np.array(d).reshape(len(src_nodes), len(sink_nodes))
//   return d


// def setup_height_arrays(src_nodes, sink_nodes):
//   src_array = np.array([node.h for node in src_nodes])
//   sink_array = np.array([node.h for node in sink_nodes])
//   return src_array, sink_array


// def solve_emd(x, y, d):
//   """
//   Solves MILP Earth Mover's Distance Problem
//   Args:
//     x (numpy.ndarray, dim = m x 1): vector of sink values to fill ("subgrade")
//     y (numpy.ndarray, dim = n x 1): vector of source values to cut ("supergrade")
//     d (numpy.ndarray, dim = n x m): distance matrix between sources and sinks i.e. d(x_i, y_j) 
//   Returns:
//     pi (numpy.ndarray, dim = n x m): matrix for volumes to transport from source y_j to x_i
//     cost (float): total cost for problem solution
//     sol_description (string): text describing some intution/insight for solution
//   """
//   # ---------------------------------------------------------------------------
//   # Create c, G, h matrices for cvx.solvers.lp()
//   n = y.shape[0]
//   m = x.shape[0]
//   # --------
//   # c: Cost vector from distance matrix, row major order
//   c = d.flatten()
//   # --------
//   # G: build up with blocks
//   # Block multipliers for y 
//   vec1m = np.ones((m, 1)).T
//   zeros1m = np.zeros((m, 1)).T
//   block1m = []
//   for i in range(n):
//     row = np.c_[np.tile(zeros1m, i), vec1m, np.tile(zeros1m, n-i-1)]
//     block1m.append(row)
//   block1m = np.vstack(block1m)
//   # Block multipliers for x
//   block1n = np.tile(np.eye(m), n)
//   # Combine blocks
//   G = np.r_[block1m, 
//             block1n,
//             -block1m,
//             -block1n,
//             -np.eye(n*m)]
//   # --------
//   # h: constraint values for inequalities
//   # Upper bound possible transport values
//   M = np.amax([np.sum(x), np.sum(y)])
//   # Check binary decision case; compare sink vs. source volumes to solve directly once instead of needing MILP search techniques (ex. exhaustive, branch & bound, etc.)
//   if np.sum(y) < np.sum(x):
//     # More sink than source; constrain by total source volume
//     sol_description = "$V_{source} < V_{sink}$ : max transport constrained by total source volume"
//     print(f"[INFO] {sol_description}, setting b=0")
//     b = 0
//   else:
//     # More source than sink, or exactly the same amout; constrain by total sink volume
//     if np.sum(y) == np.sum(x):
//       sol_description = "$V_{source} = V_{sink}$ : max cost transport, source and sink constraints equivalent"
//     else:
//       sol_description = "$V_{source} > V_{sink}$ : max transport constrained by total sink volume"
//     print(f"[INFO] {sol_description}, setting b=1")
//     b = 1
//   # Pack vector  
//   h = np.r_[y,
//              x,
//              -y + M * b,
//              -x + M * (1 - b),
//              np.zeros(n * m)]
//   # ---------------------------------------------------------------------------
//   # Solve the problem
//   c = cvx.matrix(c)
//   G = cvx.matrix(G)
//   h = cvx.matrix(h)
//   print("[INFO] Solving...\n")
//   sol = cvx.solvers.lp(c, G, h)
//   print()
//   # ---------------------------------------------------------------------------
//   # Extract solution
//   cost = sol['primal objective'] 
//   pi = np.array(sol['x']) # Transport plan
//   # Reformat and return solution
//   return pi.reshape(n,m), cost, sol_description


// def get_transport_list(policy, src_nodes, sink_nodes):
//   '''
//   List of transport plans as [(start_node_1, end_node_1, volume_1), ... (start_node_N, end_node_N, volume_N)]
//   '''
//   transport_list = []
//   zero_transport_threshold = 1e-4 # minimum threshold to consider as non-zero transport
//   for i, sink_node in enumerate(sink_nodes):
//     for j, src_node in enumerate(src_nodes):
//       new_volume = policy[j, i]
//       if new_volume > zero_transport_threshold:
//         transport_list.append([src_node, sink_node, new_volume])
  
//   return transport_list


// def find_nearest_transport(transport_list, agent_pose, visited_transport_indices=[]):
//   best_dist = np.inf
//   for j, transport in enumerate(transport_list):
//     src_node = transport[0]
//     # Compare euclidean distance between current pose and each source node
//     node_dist = (src_node.x - agent_pose.x)**2 + (src_node.y - agent_pose.y)**2
//     if node_dist < best_dist and j not in visited_transport_indices:
//       best_dist = node_dist
//       nearest_transport = transport
//       transport_idx = j

//   return nearest_transport, transport_idx


} // namespace planning
} // namespace cg
