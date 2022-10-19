#include <planning/velocity_planner.hpp>

namespace cg {
namespace planning {

// Updates the trajectory.velocity_targets field in-place
void VelocityPlanner::generateVelocityTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map) {

        cg_msgs::msg::Pose2D reference_pose = agent_pose;
        std::vector<float> velocity_targets;
        for (size_t i = 0; i < trajectory.path.size(); ++i) {

            // Transform both traj_pose and reference pose by the opposite of reference pose
            cg_msgs::msg::Point2D curr_vec_transformed = transformPoint(
                trajectory.path[i].pt, 
                create_pose2d(-reference_pose.pt.x, -reference_pose.pt.y, -reference_pose.yaw));
            
            reference_pose = trajectory.path[i];
            // If pose as positive x component, it's going forward
            // Currently using constant velocity
            if (curr_vec_transformed.x >= 0) {
                velocity_targets.push_back(constant_velocity_);
            } else {
                velocity_targets.push_back(-constant_velocity_);
            }
        }

        trajectory.set__velocity_targets(velocity_targets);

        return;

    }

} // namespace planning
} // namespace cg
