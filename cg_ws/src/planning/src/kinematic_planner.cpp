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
            lattice.push_back(generateLatticeArm(turn_radius));
            
            // Account for "negative" turn radius
            if (abs(turn_radius) > 1e-2) {
                lattice.push_back(generateLatticeArm(-turn_radius));
            }
        }

        return lattice;
    }

    std::vector<cg_msgs::msg::Pose2D> KinematicPlanner::generateLatticeArm(float turn_radius) {

        std::vector<cg_msgs::msg::Pose2D> lattice_arm;

        int num_segments = ceil(max_trajectory_length / trajectory_resolution);

        // Forward cases

        // Straight case
        if (turn_radius < 1e-2) {

            


        }


        // Backwards cases 

        // Straight case
        if (turn_radius < 1e-2) {


        }



        return lattice_arm;
    }



} // namespace planning
} // namespace cg
