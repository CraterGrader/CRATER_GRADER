#include <algorithm>

#include "motion_control/pid_controller.hpp"

namespace cg {
namespace motion_control {

PIDController::PIDController(const PIDParams &params) :
    params_(params), prev_error_(0.0), integral_error_(0.0), first_timestep_(false) {}


void PIDController::reset() {
  prev_error_ = 0.0;
  integral_error_ = 0.0;
  first_timestep_ = false;
}


double PIDController::control(double error) {
  double d_error = 0.0;
  if (first_timestep_) {
    first_timestep_ = true;
  } else {
    d_error = (error - prev_error_) / params_.dt;
  }
  prev_error_ = error;

  integral_error_ += params_.ki * error * params_.dt;
  integral_error_ = std::max(std::min(integral_error_, params_.integral_sat_max), params_.integral_sat_min);

  double u = params_.kp * error + integral_error_ + params_.kd * d_error;
  u = std::max(std::min(u, params_.output_sat_max), params_.output_sat_min);
  return u;
}


void PIDController::setGains(const double kp, const double ki, const double kd) {
  params_.kp = kp;
  params_.ki = ki;
  params_.kd = kd;
}

}  // namespace motion_control
}  // namespace cg
