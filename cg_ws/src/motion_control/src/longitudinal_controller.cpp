#include <limits>

#include "motion_control/longitudinal_controller.hpp"

namespace cg {
namespace motion_control {

LongitudinalController::LongitudinalController(const PIDParams &params) {
  velocity_controller_ = std::make_unique<PIDController>(PIDController(params));
}

void LongitudinalController::setGains(const double kp, const double ki, const double kd) {
  velocity_controller_->setGains(kp, ki, kd);
}

double LongitudinalController::computeDrive(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state,
    const size_t traj_idx, const float steer_velocity){
  // double target_velocity, curr_velocity;
  double curr_velocity = current_state.twist.twist.linear.x;

  // Get target velocity by using "closest" index
  double target_velocity = 0.0;
  if (target_trajectory.path.size() > 0) {
    target_velocity = target_trajectory.velocity_targets[traj_idx];
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // no cascaded PID necessary, low level PID working fine
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // double target_velocity = target_trajectory.path.size() ?
  //     target_trajectory.velocity_targets[getClosestPointIndex(target_trajectory, current_state)] : 0.0;
  // TODO: cur velocity is in m/s, target is currently in %fs
  // double error = target_velocity - curr_velocity;
  // double desired_drive = velocity_controller_->control(error);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  float steer_scale_factor = std::max((max_steer_speed_ - steer_velocity)/(max_steer_speed_),min_drive_speed_scaler_);

  double desired_drive = steer_scale_factor * target_velocity; 

  return desired_drive; // DEBUG
}

double LongitudinalController::scaleToDriveActuators(double desired_drive) {
  // Calculated using % full scale of steering angle [%FS / (steer angle in radians)]
  double transfer_function_qpps_to_mps = 0.003795275591;
  return (1/transfer_function_qpps_to_mps) * desired_drive;
}



}  // namespace motion_control
}  // namespace cg
