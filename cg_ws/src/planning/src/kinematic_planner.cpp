#include <planning/kinematic_planner.hpp>

namespace cg {
namespace planning {

    void KinematicPlanner::generatePath(
        std::vector<cg_msgs::msg::Pose2D> &path,
        const cg_msgs::msg::Pose2D &agent_pose,
        const cg_msgs::msg::Pose2D &goal_pose,
        const cg::mapping::Map<float> &map) {
            // Instantiate lattice
            std::vector<std::vector<cg_msgs::msg::Pose2D>> base_lattice = KinematicPlanner::generateBaseLattice();

            // Run astar search
            path = KinematicPlanner::latticeAStarSearch(agent_pose, goal_pose, map, base_lattice);

            // Note: Path validity is already checked at every step of A* Search, do we need to check again here?
            return;
        }

    std::vector<cg_msgs::msg::Pose2D> KinematicPlanner::latticeAStarSearch(
        const cg_msgs::msg::Pose2D &agent_pose,
        const cg_msgs::msg::Pose2D &goal_pose,
        const cg::mapping::Map<float> &map,
        const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice) {
        
        // TODO: Maybe stopping the planner if exceeding # iterations of lattice planning?
        
        // Ensure start is on the map
        // assert(map.pose_on_map(agent_pose)); TODO ADD

        // List of final trajectories composed from lattice primitives
        std::vector<cg_msgs::msg::Pose2D> final_path;
        
        // Note: Currently instead of cg_msgs::msg::Trajectory, we are using Pose2D but this can be changed
        std::vector<std::vector<cg_msgs::msg::Pose2D>> visited_trajectories;
        std::vector<AStarNode> visited_nodes;

        // Lower cost has higher priority (i.e. earlier in queue) - Min Heap
        // Will compare first value (cost) when calling pop()
        std::priority_queue<
            std::pair<float, AStarNode>, 
            std::vector<std::pair<float, AStarNode>>, 
            std::greater<std::pair<float, AStarNode>>> pq_nodes;
        
        // Initialize with first trajectory and node
        std::vector<cg_msgs::msg::Pose2D> start_trajectory = {agent_pose};
        AStarNode start_node = {0.0, 0, 0, agent_pose, start_trajectory};
        pq_nodes.push(std::make_pair(0.0, start_node));

        int num_iter = 0;
        while (!pq_nodes.empty()) {
            num_iter++;
            if (num_iter % 100 == 0) {
                std::cout << "# Latter Planner Iterations: " << num_iter << std::endl;
            }

            // Obtain node with the lowest f-cost and remove from queue
            AStarNode curr_node = pq_nodes.top().second;
            pq_nodes.pop();

            // Check if current node can can complete trajectory
            auto [closest_traj_pose, closest_traj_idx] = KinematicPlanner::getClosestTrajectoryPoseToGoal(curr_node.trajectory, goal_pose);
            if (KinematicPlanner::samePoseWithinThresh(closest_traj_pose, goal_pose)) {
                int curr_idx = curr_node.idx;

                // Add final trajectory to 
                if (visited_trajectories.empty()) {
                    visited_trajectories.push_back(curr_node.trajectory);
                    visited_nodes.push_back(curr_node);
                }

                // Reverse last segment
                std::vector<cg_msgs::msg::Pose2D> traj_copy = visited_trajectories[curr_idx];
                std::reverse(traj_copy.begin(), traj_copy.end());
                final_path.insert(final_path.end(), 
                                  traj_copy.begin() + (traj_copy.size() - closest_traj_idx), 
                                  traj_copy.end());
                
                // Add trajectory segment in reverse order
                curr_idx = visited_nodes[curr_idx].parent_idx;
                while (curr_idx != 0) {
                    
                    // Reverse current trajectory segment to push it on backwards
                    std::vector<cg_msgs::msg::Pose2D> traj_copy = visited_trajectories[curr_idx];
                    std::reverse(traj_copy.begin(), traj_copy.end());
                    final_path.insert(final_path.end(), traj_copy.begin(), traj_copy.end());

                    curr_idx = visited_nodes[curr_idx].parent_idx;
                }

                // Since we have been pushing onto final_path backwards, reverse it
                std::reverse(final_path.begin(), final_path.end());
                
                // Break since found end point
                break;
            }

            // Obtain lattice from current pose
            std::vector<std::vector<cg_msgs::msg::Pose2D>> lattice_trajectories = 
                transformLatticeToPose(base_lattice, curr_node.pose);

            // Filter out nonvalid trajectories
            std::vector<std::vector<cg_msgs::msg::Pose2D>> valid_trajectories;
            for (auto trajectory : lattice_trajectories) {
                if (KinematicPlanner::isValidTrajectory(trajectory, map)) {
                    valid_trajectories.push_back(trajectory);
                }
            }

            // Calculate trajectory heuristic h(s') for valid trajectories
            std::vector<float> trajectories_heuristic = KinematicPlanner::trajectories_heuristic(valid_trajectories, goal_pose);
                
            // Get trajectory costs c(s,s') for valid trajectories
            std::vector<float> trajectories_costs;
            for (auto trajectory: valid_trajectories){
                trajectories_costs.push_back(KinematicPlanner::calculateTopographyCost(trajectory, map));
            }

            // Calculate next nodes on all valid trajectories
            for (size_t traj_idx = 0; traj_idx < valid_trajectories.size(); ++traj_idx) {
                
                // Skip if trajectory includes points out of bounds
                if (!KinematicPlanner::isValidTrajectory(valid_trajectories[traj_idx], map)) continue;

                cg_msgs::msg::Pose2D end_of_cur_traj_pose = valid_trajectories[traj_idx].back();

                // Skip if nose pose is close to another node that has been visited
                bool skip_new_node = false;
                for (auto visited_node : visited_nodes) {
                    if (KinematicPlanner::samePoseWithinThresh(end_of_cur_traj_pose, visited_node.pose)) {
                        skip_new_node = true;
                        break;
                    }
                }
                if (skip_new_node) continue;

                // Update node costs g(s') = g(s) + c(s,s')
                float succ_g_cost = curr_node.g_cost + trajectories_costs[traj_idx];

                // Create new child node
                AStarNode succ_node = {succ_g_cost, 
                                       static_cast<int>(visited_trajectories.size()), // Current pose idx
                                       curr_node.idx, // Parent pose idx
                                       end_of_cur_traj_pose, // Current Pose
                                       valid_trajectories[traj_idx]}; //
                visited_trajectories.push_back(valid_trajectories[traj_idx]);
                visited_nodes.push_back(succ_node);

                // Add new poses to pqueue using f(s') = g(s') + h(s')
                float f_cost = succ_g_cost + trajectories_heuristic[traj_idx];
                pq_nodes.push(std::make_pair(f_cost, succ_node));
            }
        }

        if (final_path.empty()) {
            std::cout << "[WARN] No Solution Found, returning empty path..." << std::endl;
        }

        std::cout << "Final planner lattice iterations" << num_iter << std::endl;

        return final_path;
    }
    

    bool KinematicPlanner::samePoseWithinThresh(
        const cg_msgs::msg::Pose2D &trajectory_end_pose,
        const cg_msgs::msg::Pose2D &goal_pose) {
            return (
                cg::planning::euclidean_distance(trajectory_end_pose.pt, goal_pose.pt) < pose_position_equality_threshold &&
                abs(trajectory_end_pose.yaw - goal_pose.yaw) < pose_yaw_equality_threshold);
    }

    std::vector<std::vector<cg_msgs::msg::Pose2D>> KinematicPlanner::generateBaseLattice() {

        assert(turn_radii_min > 0 && turn_radii_max > 0);

        // Generate turn_radii vector
        std::vector<float> turn_radii;
        float cur_radii = turn_radii_min;
        while (cur_radii <= turn_radii_max) {
            turn_radii.push_back(cur_radii);
            cur_radii += turn_radii_resolution;
        }

        std::vector<std::vector<cg_msgs::msg::Pose2D>> lattice;
        for (float turn_radius: turn_radii) {
            // Forwards right
            lattice.push_back(generateLatticeArm(turn_radius, true, true));
            // Forwards left
            lattice.push_back(generateLatticeArm(turn_radius, true, false));
            // Backwards right
            lattice.push_back(generateLatticeArm(turn_radius, false, true));
            // Backwards left
            lattice.push_back(generateLatticeArm(turn_radius, false, false));
        }
        // Going straight, forwards/backwards
        lattice.push_back(generateLatticeArm(0, true, false));
        lattice.push_back(generateLatticeArm(0, false, false));
        return lattice;
        
        }

    std::vector<cg_msgs::msg::Pose2D> KinematicPlanner::generateLatticeArm(
        float turn_radius, bool forwards, bool right) {

        // Since left/right is handled with flags, turn radius should always be positive
        assert(turn_radius > 0);

        std::vector<cg_msgs::msg::Pose2D> lattice_arm;
        int num_segments = ceil(max_trajectory_length / trajectory_resolution);
        float x,y,yaw;
        x = y = yaw = 0.0;

        if (!forwards) max_trajectory_length *= -1;

        // Straight (forwards and backwards)
        if (turn_radius < 1e-2) {
            float incremental_movement = max_trajectory_length / num_segments;
            if (!forwards) incremental_movement *= -1;

            for (int n = 0; n <= num_segments; ++n) {
                x += incremental_movement;
                cg_msgs::msg::Pose2D pose = cg::planning::create_pose2d(x, y, yaw);
                lattice_arm.push_back(pose);
            }
        }
        // Right (forwards and backwards)
        else if (right) {
            float start_rad = M_PI/2;
            float stop_rad = (-max_trajectory_length / turn_radius) + start_rad;
            float rad_increment = (stop_rad - start_rad) / num_segments;

            float cur_rad = start_rad;
            for (int n = 0; n <= num_segments; ++n) {
                x = turn_radius * cos(cur_rad);
                y = turn_radius * sin(cur_rad) - turn_radius;
                yaw = M_PI + atan2(-cos(cur_rad), sin(cur_rad));

                cg_msgs::msg::Pose2D pose= cg::planning::create_pose2d(x, y, yaw);
                lattice_arm.push_back(pose);
                cur_rad += rad_increment;
            }
        // Left (forwards & backwards)
        } else {
            float start_rad = -M_PI/2;
            float stop_rad = (max_trajectory_length / turn_radius) + start_rad;
            float rad_increment = (stop_rad - start_rad) / num_segments;

            float cur_rad = start_rad;
            for (int n = 0; n <= num_segments; ++n) {
                x = turn_radius * cos(cur_rad);
                y = turn_radius * sin(cur_rad) + turn_radius;
                yaw = atan2(-cos(cur_rad), sin(cur_rad));

                cg_msgs::msg::Pose2D pose = cg::planning::create_pose2d(x, y, yaw);
                lattice_arm.push_back(pose);
                cur_rad += rad_increment;
            }
        }

        return lattice_arm;
    }

    std::pair<cg_msgs::msg::Pose2D, int> KinematicPlanner::getClosestTrajectoryPoseToGoal(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose) {

        float closest_distance = INFINITY;
        float dist;
        int closest_idx = -1;
        for (size_t i = 0; i < trajectory.size(); ++i) {
            dist = cg::planning::euclidean_distance(trajectory[i].pt, goal_pose.pt);
            if (dist < closest_distance) {
                closest_distance = dist;
                closest_idx = i;
            }
        }
        return std::make_pair(trajectory[closest_idx], closest_idx);
    }

    std::vector<std::vector<cg_msgs::msg::Pose2D>> KinematicPlanner::transformLatticeToPose(
        const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice,
        const cg_msgs::msg::Pose2D &current_pose) {

        std::vector<std::vector<cg_msgs::msg::Pose2D>> transformed_lattice;   
        for (std::vector<cg_msgs::msg::Pose2D> trajectory : base_lattice) {

            std::vector<cg_msgs::msg::Pose2D> transformed_trajectory;
            for (cg_msgs::msg::Pose2D pose : trajectory) {

                transformed_trajectory.push_back(transformPose(pose, current_pose));
            }
            transformed_lattice.push_back(transformed_trajectory);
        }

        return transformed_lattice;
    }


    bool KinematicPlanner::isValidTrajectory(
        const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
        const cg::mapping::Map<float> &map) {

        // Check if all points of vehicle are within bounds
        for (cg_msgs::msg::Pose2D pose : trajectory) {
            // Check if point is in limits of map
            if (!map.validPoint(pose.pt)) return false;
        }
        return true;
        // TODO: Do we need footprint information to make this effective?
        // Maybe that's in the map data structure?
    }

    float KinematicPlanner::calculateTopographyCost(
        const std::vector<cg_msgs::msg::Pose2D> &trajectory,
        const cg::mapping::Map<float> &map) {

            float topography_cost = 0;
            for (cg_msgs::msg::Pose2D pose : trajectory) {

                size_t pose_idx = map.continousCoordsToCellIndex(pose.pt);
                topography_cost += abs(map.getDataAtIdx(pose_idx));
            }

            // Weight topography cost
            float weighted_topography_cost = topography_weight * topography_cost;

            return weighted_topography_cost;
        }

    std::vector<float> KinematicPlanner::trajectories_heuristic(
        const std::vector<std::vector<cg_msgs::msg::Pose2D>> &trajectories, 
        const cg_msgs::msg::Pose2D &goal_pose) {
            std::vector<float> trajectories_heuristic;
            for (std::vector<cg_msgs::msg::Pose2D> trajectory : trajectories) {
                trajectories_heuristic.push_back(euclidean_distance(trajectory.back().pt, goal_pose.pt));
            }
            // Distance between goal pose and final point of trajectory
            return trajectories_heuristic;
        }


} // namespace planning
} // namespace cg
