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
    double closest_euclidian_error = std::numeric_limits<double>::infinity();
    double closest_heading_error;
    for (size_t i = 0; i < target_trajectory.path.size(); ++i) {
        double dist = cg::planning::euclidean_distance(target_trajectory.path[i].pt, cur_pose.pt);
        
        // If trajectory point closer, then update considered target point
        if (dist < closest_euclidian_error) {
            
            // Transform both traj_pose and reference pose by the opposite of reference pose
            cg_msgs::msg::Point2D transformed_error = cg::planning::transformPointGlobalToLocal(cur_pose.pt, target_trajectory.path[i]);

            closest_euclidian_error = dist;
            closest_cross_track_error = transformed_error.y;
            
            closest_heading_error = cg::planning::smallest_angle_difference(target_trajectory.path[i].yaw, cur_pose.yaw);
            prev_traj_idx_ = i;
        }
    }

    debug_.cross_track_err = closest_cross_track_error;
    debug_.heading_err = closest_heading_error;

    // Considering only planar velocity since trajectory is planar
    double current_velocity = std::sqrt(
      std::pow(current_state.twist.twist.linear.x, 2) + 
      std::pow(current_state.twist.twist.linear.y, 2));

    // Compute stanley control law
    double desired_steer = LateralController::stanleyControlLaw(
      closest_heading_error,
      closest_cross_track_error,
      current_velocity);
    // Scale the desired steer angle to actuator steer position
    return scaleToSteerActuators(desired_steer);
}

double LateralController::stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const {

    double steer_correct_angle = std::atan2(k_ * cross_track_err, velocity + stanley_softening_constant_);

    return heading_err + steer_correct_angle;
    }
double LateralController::scaleToSteerActuators(double desired_steer){
  // Calculated using % full scale of steering angle [%FS / (steer angle in radians)]
  double transfer_function_to_steer_position = -360.712;

  return transfer_function_to_steer_position * desired_steer;
}

void LateralController::resetPrevTrajIdx() {
    prev_traj_idx_ = 0;
}

} // namespace motion_control
} // namespace cg
