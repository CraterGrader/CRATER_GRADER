#include "motion_control/lateral_controller.hpp"

namespace cg {
namespace motion_control {


LateralController::LateralController(double k, double stanley_softening_constant) :
    k_(k),
    stanley_softening_constant_(stanley_softening_constant) {}


double LateralController::computeSteer(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state) {

    tf2::Quaternion q = tf2::Quaternion(
        current_state.pose.pose.orientation.x,
        current_state.pose.pose.orientation.y,
        current_state.pose.pose.orientation.z,
        current_state.pose.pose.orientation.w);

    cg_msgs::msg::Pose2D cur_pose = cg::planning::create_pose2d(
        current_state.pose.pose.position.x,
        current_state.pose.pose.position.y,
        tf2::getYaw(q));

    // Get closest trajectory reference point
    double closest_cross_track_error;
    double closest_traj_distance = INFINITY;
    double closest_heading_error;
    double closest_velocity_target;
    for (size_t i = 0; i < target_trajectory.path.size(); ++i) {
        double dist = cg::planning::euclidean_distance(target_trajectory.path[i].pt, cur_pose.pt);
        
        // If trajectory point closer, then update considered target point
        if (dist < closest_traj_distance) {
            closest_traj_distance = dist;
            closest_cross_track_error = target_trajectory.path[i].pt.x - cur_pose.pt.x;
            closest_heading_error = target_trajectory.path[i].yaw - cur_pose.yaw;
            closest_velocity_target = target_trajectory.velocity_targets[i];
        }
    }

    // Compute stanley control law
    double desired_steer = LateralController::stanleyControlLaw(
        closest_heading_error,
        closest_cross_track_error,
        closest_velocity_target);

    return desired_steer;
    }


double LateralController::stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const {

    double steer_correct_angle = std::atan2(
        k_ * -cross_track_err, 
        velocity + stanley_softening_constant_);

    return heading_err + steer_correct_angle;
    }


} // namespace motion_control
} // namespace cg

