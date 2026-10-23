#pragma once

#include <Eigen/Dense>

class KalmanFilter {

public:
  KalmanFilter(
    const Eigen::MatrixXd& A, // System dynamics.
    const Eigen::MatrixXd& B, // Control.
    const Eigen::MatrixXd& C, // Output.
    const Eigen::MatrixXd& P, // Estimate error covariance.
    const Eigen::MatrixXd& R, // Measurement noise covariance.
    const Eigen::MatrixXd& Q  // Process noise covariance.
    );
  ~KalmanFilter();

  void init();
  void init(const Eigen::VectorXd& x);
  void predict(const Eigen::VectorXd& u);
  void update(const Eigen::VectorXd& y);
  Eigen::VectorXd get_state() const;

  // Time-varying noise (e.g. scaled by contact state every step); same size as at construction
  void set_Q(const Eigen::MatrixXd& Q);
  void set_R(const Eigen::MatrixXd& R);
  // Overwrite the estimate without touching P (init() resets P)
  void set_state(const Eigen::VectorXd& x);
  const Eigen::MatrixXd& get_P() const;

private:
  Eigen::MatrixXd A, B, C, P, R, Q;
  Eigen::MatrixXd P0;   // initial P.
  Eigen::MatrixXd K;    // kalman gain.
  Eigen::MatrixXd I;    // unit matrix.

  Eigen::VectorXd x_hat; // estimated state.
};
