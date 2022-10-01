#include <planning/kinematic_planner.hpp>

namespace cg {
namespace planning {

    // Construct Kinematic Planner
    KinematicPlanner::KinematicPlanner() : 
        goal_pose_distance_threshold(1e-5), 
        turn_radii_min(0.8), 
        turn_radii_max(1.6), 
        turn_radii_resolution(0.4),
        max_trajectory_length(0.4),
        trajectory_resolution(0.05),
        pose_position_equality_threshold(0.05),
        pose_position_equality_threshold(deg2rad(5)) {};

    void generatePath(
        std::vector<cg_msgs::msg::Pose2D> &path,
        const cg_msgs::msg::Pose2D &agent_pose,
        const cg::mapping::SiteMap &map) {
            // TODO

            // Instantiate lattice

            // Run astar search

            // If valid traj found, return


        }

    std::vector<cg_msgs::msg::Pose2D> lattice_astar_search(
        const cg_msgs::msg::Pose2D &agent_pose,
        const cg_msgs::msg::Pose2D &goal_pose,
        const cg::mapping::SiteMap &map,
        const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice) {
            // TODO

            // Full Astar, follow prototype

            // Use AStarNode struct

            // Create starting nodes in priority queue (find valid datatstructure)

            // Perform standard Astar, where children node come from lattice generated

            // Add topography cost and heursitic to node cost


            // returns valid trajectory
            // if cannot find...
            // Add some form of horizon to stop if in loop??

            // If no traj found, return empty vector

        }
    

    bool KinematicPlanner::samePoseWithThresh(
        const cg_msgs::msg::Pose2D &trajectory_end_pose,
        const cg_msgs::msg::Pose2D &goal_pose) {
            return euclidean_distance(trajectory_end_pose, goal_pose) <= goal_pose_distance_threshold;
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

    std::vector<cg_msgs::msg::Pose2D> KinematicPlanner::generateLatticeArm(float turn_radius, bool forwards, bool right) {

        // Since left/right is handled with flags, turn radius should always be positive
        assert(turn_radius > 0);

        std::vector<cg_msgs::msg::Pose2D> lattice_arm;
        int num_segments = ceil(max_trajectory_length / trajectory_resolution);
        float x,y,yaw;
        x = y = yaw = 0.0;

        float trajectory_arc = max_trajectory_length;
        if (!forwards) max_trajectory_length *= -1;

        // Straight (forwards and backwards)
        if (turn_radius < 1e-2) {
            float incremental_movement = max_trajectory_length / num_segments;
            if (!forwards) incremental_movement *= -1;

            for (int n = 0; n <= num_segments; ++n) {
                x += incremental_movement;
                cg_msgs::msg::Pose2D pose = create_pose2d(x, y, yaw);
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

                cg_msgs::msg::Pose2D pose = create_pose2d(x, y, yaw);
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

                cg_msgs::msg::Pose2D pose = create_pose2d(x, y, yaw);
                lattice_arm.push_back(pose);
                cur_rad += rad_increment;
            }
        }

        return lattice_arm;
    }

    std::pair<cg_msgs::msg::Pose2D, int> getClosestTrajectoryPoseToGoal(
    const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
    const cg_msgs::msg::Pose2D &goal_pose) {

        float closest_distance = INFINITY;
        float dist;
        int closest_idx = -1;
        for (int i = 0; i < trajectory.size(); ++i) {
            dist = euclidean_distance(trajectory[i], goal_pose);
            if (dist < closest_distance) {
                closest_distance = dist;
                closest_idx = i;
            }
        }
        return std::make_pair(trajectory[closest_idx], closest_idx);
    }

    std::vector<std::vector<cg_msgs::msg::Pose2D>> KinematicPlanner::transformLatticeToPose(
        const std::vector<std::vector<cg_msgs::msg::Pose2D>> &base_lattice,
        const cg_msgs::msg::Pose2D &current_pose) 
    {

        std::vector<std::vector<cg_msgs::msg::Pose2D>> transformed_lattice;   
        for (std::vector<cg_msgs::msg::Pose2D> trajectory : base_lattice) {

            std::vector<cg_msgs::msg::Pose2D> transformed_trajectory;
            for (cg::msgs::msg::Pose2D pose: trajectory) {

                transformed_trajectory.push_back(transformPose(pose, current_pose));
            }
            transformed_lattice.push_back(transformed_trajectory);
        }

        return transformed_lattice;
    }

    bool samePoseWithinThresh(
        const cg_msgs::msg::Pose2D &trajectory_end_pose,
    const cg_msgs::msg::Pose2D &goal_pose) {
        return euclidean_distance(trajectory_end_pose, goal_pose) <= goal_pose_distance_threshold; 
    }

    bool isValidTrajectory(
        const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
        const cg::mapping::SiteMap &map) {

            // TODO

            // Access map data to find bounds
            // Check if all points of vehicle are within bounds
            // Do we need footprint information to make this effective?
                // Maybe that's in the map data structure?

        }

    std::vector<float> calculateTopographyCost(
        const std::vector<cg_msgs::msg::Pose2D> &trajectory,
        const cg::mapping::SiteMap &map)
        {
            // TODO
            // Compute summed heights of map along trajectory
        }

    float trajectory_heuristic(
        const std::vector<cg_msgs::msg::Pose2D> &trajectory, 
        const cg_msgs::msg::Pose2D &goal_pose)
        {
            // TODO
            // Distance between goal pose and final point of trajectory
        }

    bool posesWithinThresh(
        const cg_msgs::msg::Pose2D &pose,
        const std::vector<cg_msgs::msg::Pose2D> &goal_pose) {

            return (
                euclidean_distance(pose.pt, goal_pose.pt) < pose_position_equality_threshold && 
                abs(pose.yaw - goal_pose.yaw) < pose_yaw_equality_threshold);

        }


} // namespace planning
} // namespace cg
