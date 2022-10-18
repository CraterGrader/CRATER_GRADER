#include <planning/velocity_planner.hpp>

namespace cg {
namespace planning {

// Updates the trajectory.velocity_targets field in-place
void VelocityPlanner::generateVelocityTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map) {

        // Currently using constant velocity
        std::vector<float> velocity_targets(trajectory.path.size(), constant_velocity_);
        trajectory.velocity_targets = velocity_targets;
        return;
    }

} // namespace planning
} // namespace cg
