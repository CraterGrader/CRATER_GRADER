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

// TODO this should probably be a util function, pretty much the same thing is done for lateral controller
int LongitudinalController::getClosestPointIndex(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state)
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

  cg_msgs::msg::Point2D cur_point = cg::planning::create_point2d(
      current_state.pose.pose.position.x,
      current_state.pose.pose.position.y);

  double min_dist = std::numeric_limits<double>::infinity();
  int min_idx = -1;
  for (size_t i = prev_traj_idx_; i < target_trajectory.path.size(); ++i) {
    double dist = cg::planning::euclidean_distance(target_trajectory.path[i].pt, cur_pose.pt);
    if (dist < min_dist) {
        min_dist = dist;
        min_idx = i;
        prev_traj_idx_ = i;
    }
  }

  // check if we are at last index
  if (prev_traj_idx_ + 1 == target_trajectory.path.size()){
    return prev_traj_idx_;
  }

  // make local frame using path and yaw
  cg_msgs::msg::Point2D point_rel_to_traj = cg::planning::transformPointGlobalToLocal(cur_point, target_trajectory.path[prev_traj_idx_]);

  // get x componant of pose in traj frame
  double x_of_point_in_traj_frame = point_rel_to_traj.x;

  // get velocity componant
  double velocity_of_traj = target_trajectory.velocity_targets[prev_traj_idx_];

  // if product is positive 
  // remove the index, because we are in front of a forward drive pose, or behind a reverse drive node
  if ((velocity_of_traj * x_of_point_in_traj_frame) > 0){
    prev_traj_idx_ += 1;
  }
    
  return prev_traj_idx_;
}

double LongitudinalController::computeDrive(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state) {
  // double target_velocity, curr_velocity;
  double curr_velocity = current_state.twist.twist.linear.x;

  double target_velocity = target_trajectory.path.size() ?
      target_trajectory.velocity_targets[getClosestPointIndex(target_trajectory, current_state)] : 0.0;

  // TODO: cur velocity is in m/s, target is currently in %fs
  // double error = target_velocity - curr_velocity;
  // double desired_drive = velocity_controller_->control(error);
  double desired_drive = target_velocity; // DEBUG PID  on PID is non relevent for our usecase

  // return scaleToDriveActuators(desired_drive);
  return desired_drive; // DEBUG
}

double LongitudinalController::scaleToDriveActuators(double desired_drive) {
  // Calculated using % full scale of steering angle [%FS / (steer angle in radians)]
  double transfer_function_qpps_to_mps = 0.003795275591;
  return (1/transfer_function_qpps_to_mps) * desired_drive;
}

void LongitudinalController::resetPrevTrajIdx() {
  prev_traj_idx_ = 0;
}


}  // namespace motion_control
}  // namespace cg
