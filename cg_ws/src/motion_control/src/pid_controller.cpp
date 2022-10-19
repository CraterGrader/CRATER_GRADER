#include "motion_control/pid_controller.hpp"

namespace cg {
namespace motion_control {

PIDController::PIDController(const PIDParams &params) :
    params_(params), prev_error_(0.0), integral_error_(0.0) {}

void PIDController::reset() {
  prev_error_ = 0.0;
  integral_error_ = 0.0;
}

void PIDController::setGains(const double kp, const double ki, const double kd) {
  params_.kp = kp;
  params_.ki = ki;
  params_.kd = kd;
}

}  // namespace motion_control
}  // namespace cg
