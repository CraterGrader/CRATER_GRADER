#include "localization/kalman_filter_linear.hpp"

cg::localization::KalmanFilterLinear::KalmanFilterLinear() {}

cg::localization::KalmanFilterLinear::KalmanFilterLinear(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &H,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P)
    : A(A), H(H), Q(Q), R(R), P0(P), initialized(false),
      I(A.rows(), A.rows()), x_hat(A.rows()), x_hat_new(A.rows())
{
  I.setIdentity();
}

void cg::localization::KalmanFilterLinear::init(const Eigen::VectorXd &x0)
{
  x_hat = x0;
  P = P0;
  initialized = true;
}

void cg::localization::KalmanFilterLinear::init()
{
  x_hat.setZero();
  P = P0;
  initialized = true;
}

void cg::localization::KalmanFilterLinear::predict()
{
  // Predict next state and uncertainty
  x_hat_new = A * x_hat;
  P = A * P * A.transpose() + Q;
}


void cg::localization::KalmanFilterLinear::update(const Eigen::VectorXd &z)
{
  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");

  // Calculate Kalman Gain
  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Update state and uncertainty
  x_hat_new += K * (z - H * x_hat_new);
  P = (I - K * H) * P;

  // Set next state
  x_hat = x_hat_new;
}