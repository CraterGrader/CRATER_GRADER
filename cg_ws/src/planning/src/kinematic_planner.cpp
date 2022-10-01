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
        trajectory_resolution(0.05) {};

    bool KinematicPlanner::samePoseWithThresh(
        const cg_msgs::msg::Pose2D &trajectory_end_pose,
        const cg_msgs::msg::Pose2D &goal_pose) {
            return euclidean_distance(trajectory_end_pose, goal_pose) <= goal_pose_distance_threshold;
    }

    std::vector<std::vector<cg_msgs::msg::Pose2D>> KinematicPlanner::generateBaseLattice() {

        // Generate turn_radii vector
        std::vector<float> turn_radii;
        float cur_radii = turn_radii_min;
        while (cur_radii <= turn_radii_max) {
            turn_radii.push_back(cur_radii);
            cur_radii += turn_radii_resolution;
        }

        std::vector<std::vector<cg_msgs::msg::Pose2D>> lattice;
        for (float turn_radius: turn_radii) {
            lattice.push_back(generateLatticeArm(turn_radius, true, true));
            lattice.push_back(generateLatticeArm(turn_radius, true, false));
            
            // Account for "negative" turn radius
            if (abs(turn_radius) > 1e-2) {
                lattice.push_back(generateLatticeArm(turn_radius, false, true));
                lattice.push_back(generateLatticeArm(turn_radius, false, false));
            }
        }

        return lattice;
    }

    std::vector<cg_msgs::msg::Pose2D> KinematicPlanner::generateLatticeArm(float turn_radius, bool forwards, bool right) {

        // Since left/right is handled with flags, turn radius should always be positive
        assert(turn_radius > 0);

        std::vector<cg_msgs::msg::Pose2D> lattice_arm;
        int num_segments = ceil(max_trajectory_length / trajectory_resolution);
        float x,y,yaw;
        x = y = yaw 0.0;

        float trajectory_arc = max_trajectory_length;
        if (!forwards) max_trajectory_length *= -1;

        // Straight (forwards and backwards)
        if (abs(turn_radius) < 1e-2) {
            float incremental_movement = static_cast<float>(max_trajectory_length / num_segments);
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



} // namespace planning
} // namespace cg
