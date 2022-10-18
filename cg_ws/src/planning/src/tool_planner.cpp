#include <planning/tool_planner.hpp>
#include <iostream>

namespace cg {
namespace planning {

// Updates the trajectory.tool_positions field in-place
void ToolPlanner::generateToolTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map) {

        // Currently using basic autograder heuristic. 
        // When trajectory is going forward (based on adjacent poses)
        // Then blade is lifted

        // Forward is determined by transforming poses by the yaw value
        cg_msgs::msg::Pose2D reference_pose = agent_pose;
        std::vector<float> tool_positions;
        for (size_t i = 0; i < trajectory.path.size(); ++i) {

            // Transform both traj_pose and reference pose by the opposite of reference pose
            cg_msgs::msg::Point2D curr_vec_transformed = transformPoint(
                trajectory.path[i].pt, 
                create_pose2d(-reference_pose.pt.x, -reference_pose.pt.y, -reference_pose.yaw));
            
            reference_pose = trajectory.path[i];

            // If pose as positive x component, it's going forward
            if (curr_vec_transformed.x >= 0) {
                tool_positions.push_back(design_blade_height_);
            } else {
                tool_positions.push_back(raised_blade_height_);
            }
        }

        trajectory.set__tool_positions(tool_positions);

        return;

    }

} // namespace planning
} // namespace cg
