#include "motion_control/lateral_controller.hpp"
#include <iostream> // DEBUG

namespace cg {
namespace motion_control {

LateralController::LateralController(double k, double stanley_softening_constant) :
    k_(k),
    stanley_softening_constant_(stanley_softening_constant) {}

double LateralController::computeSteer(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state,
    const size_t traj_idx)
{

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
    cg_msgs::msg::Point2D transformed_error = cg::planning::transformPointGlobalToLocal(cur_pose.pt, target_trajectory.path[traj_idx]);
    double closest_cross_track_error = transformed_error.y;
    // double closest_euclidian_error = std::numeric_limits<double>::infinity();
    std::cout << " ++++++ target yaw: " << target_trajectory.path[traj_idx].yaw << ", current yaw: " << cur_pose.yaw << std::endl;
    double closest_heading_error = cg::planning::smallest_angle_difference_signed(target_trajectory.path[traj_idx].yaw, cur_pose.yaw);
    
    debug_.cross_track_err = closest_cross_track_error;
    debug_.heading_err = closest_heading_error;

    // Considering only planar velocity since trajectory is planar
    double current_velocity = std::sqrt(
      std::pow(current_state.twist.twist.linear.x, 2) + 
      std::pow(current_state.twist.twist.linear.y, 2));

    // Compute stanley control law
    double desired_steer = LateralController::stanleyControlLaw(closest_heading_error, closest_cross_track_error, current_velocity);

    // Reverse driving needs to flip sign
    if (target_trajectory.velocity_targets[traj_idx] < 0) {
        desired_steer = -desired_steer;
    }

    // Scale the desired steer angle to actuator steer position
    return scaleToSteerActuators(desired_steer);
}

double LateralController::stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const {

    // double steer_correct_angle = std::atan2(k_ * cross_track_err, velocity + stanley_softening_constant_);
    double steer_correct_angle = -1 * std::atan2(k_ * cross_track_err, 1); // Positive cross track error should result in a negative steering control because of z-up coordinate system

    std::cout << " +++++++++ heading_err: " << heading_err << ", cross_track_err: " << cross_track_err << ", velocity: " << velocity << ", steer_correct_angle: " << steer_correct_angle << std::endl;
    return (heading_gain_ * heading_err) + steer_correct_angle; 
    }
double LateralController::scaleToSteerActuators(double desired_steer){
  // Calculated using % full scale of steering angle [%FS / (steer angle in radians)]
  double transfer_function_to_steer_position = 360.712;

  return transfer_function_to_steer_position * desired_steer;
}

} // namespace motion_control
} // namespace cg
