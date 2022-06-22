#pragma once
#include <Eigen/Dense>

/*
 * Modified from: https://github.com/hmartiro/kalman-cpp
 * Also helpful: https://www.kalmanfilter.net/multiExamples.html
 */

namespace cg
{
  namespace localization
  {

    class KalmanFilterLinear {
      public:
        /**
         * Create a Kalman filter with the specified matrices. The time step is assumed to remain constant.
         *   A - System dynamics matrix
         *   H - Measurement observation matrix
         *   Q - Process noise covariance
         *   R - Measurement noise covariance
         *   P - Estimate error covariance
         */
        KalmanFilterLinear(
            const Eigen::MatrixXd &A,
            const Eigen::MatrixXd &H,
            const Eigen::MatrixXd &Q,
            const Eigen::MatrixXd &R,
            const Eigen::MatrixXd &P);

        KalmanFilterLinear();

        /**
         * Initialize the filter with initial states as zero.
         */
        void init();

        /**
         * Initialize the filter with a guess for initial states.
         */
        void init(const Eigen::VectorXd &x0);

        /**
         * Predict the next state.
         */
        void predict();

        /**
         * Update the estimated state based on measured values.
         */
        void update(const Eigen::VectorXd &z);

        /**
         * Return the current state and time.
         */
        Eigen::VectorXd state() { return x_hat; };

      private:
        // Matrices for computation
        Eigen::MatrixXd A, H, Q, R, P, K, P0;

        // Is the filter initialized?
        bool initialized;

        // n-size identity
        Eigen::MatrixXd I;

        // Estimated states
        Eigen::VectorXd x_hat, x_hat_new;
    };

  } // namespace localization
} // namespace cg