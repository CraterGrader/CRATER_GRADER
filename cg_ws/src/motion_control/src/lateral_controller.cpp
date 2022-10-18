#include "motion_control/lateral_controller.hpp"
#include <cmath> // arctangent
#include "planning/common.hpp"

namespace cg {
namespace motion_control {


LateralController::LateralController(double k, double stanley_softening_constant) :
    double_k(k),
    stanley_softening_constant_(stanley_softening_constant) {};


double LateralController::computeSteer(
    // TODO should this be refactored to take a single TrajectoryPoint?
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state) {

    tf2::Quaternion q = Quaternion(
        current_state.pose.pose.orientation.x,
        current_state.pose.pose.orientation.y,
        current_state.pose.pose.orientation.z,
        current_state.pose.pose.orientation.w);

    cg_msgs::msg::Pose2D cur_pose = cg::planning::create_pose2d(
        current_state.pose.pose.position.x,
        current_state.pose.pose.position.y,
        q.getYaw());

    // Get closest trajectory reference point
    double closest_cross_track_error;
    double closest_heading_error = INFINITY;
    for (int i = 0; i < target_trajectory.size(); ++i) {
        double dist = cg::planning::euclidean_distance(target_trajectory[i].pt, cur_pose.pt);
        if (dist < closest_point_dist) {
            closest_cross_track_error = dist;double
            closest_heading_error = target_trajectory[i].yaw - cur_pose.yaw;
        }
    }

    // Get velocity from nav_msgs odometry message, in base_link_frame
    double velocity = current_state.twist.twist.linear.x;

    // Compute stanley control law
    return LateralController::stanleyControlLaw(
        closest_heading_error,
        closest_cross_track_error,
        velocity);
    }


double LateralController::stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const {

    double steer_correct_angle = std::atan2(
        k_ * cross_track_err, 
        velocity + stanley_softening_constant_);

    return heading_err + steer_correct_angle;
    };


} // namespace motion_control
} // namespace cg

