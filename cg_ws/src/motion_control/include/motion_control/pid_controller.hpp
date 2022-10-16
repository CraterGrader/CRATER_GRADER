#pragma once

namespace cg {
namespace motion_control {

struct PIDParams {
  double kp;
  double ki;
  double kd;
  double dt;
  double integral_sat_min;
  double integral_sat_max;
  double output_sat_min;
  double output_sat_max;
};

class PIDController {
public:
  PIDController() {}
  PIDController(const PIDParams &params);
  // Computes control command given error
  double control(double error);
  // Clears integral buildup, previous error states, etc.
  void reset();
  // Sets PID gains dynamically e.g. for gain scheduling
  void setGains(const double kp, const double ki, const double kd);
};

}  // namespace motion_control
}  // namespace cg