#include <planning/tool_planner.hpp>

namespace cg {
namespace planning {

// Updates the trajectory.tool_positions field in-place
void ToolPlanner::generateToolTargets(cg_msgs::msg::Trajectory &trajectory,
                                      const cg_msgs::msg::Pose2D &agent_pose,
                                      const cg::mapping::Map<float> &map) {

  // Currently using basic autograder heuristic.
  // When trajectory is going forward (based on adjacent poses)
  // Then blade is lifted

  (void)agent_pose; // Silence unused errors
  (void)map;        // Silence unused errors

  // Forward is determined by transforming poses by the yaw value
  std::vector<float> tool_positions;
  for (float velocity_target : trajectory.velocity_targets) {
    // std::cout << "Current velocity: " << velocity_target << std::endl; //
    // DEBUG

    if (autograder_enabled_) {
      // If pose has positive x component, it's going forward
      if (velocity_target >= 0) {
        tool_positions.push_back(design_blade_height_);
      } else {
        tool_positions.push_back(raised_blade_height_);
      }
    } else {
      // Don't lower blade if autograder is disabled
      tool_positions.push_back(raised_blade_height_);
    }
  }

  trajectory.set__tool_positions(tool_positions);

  return;
}

} // namespace planning
} // namespace cg
