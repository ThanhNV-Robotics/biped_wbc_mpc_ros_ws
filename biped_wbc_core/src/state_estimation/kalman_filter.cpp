#include "biped_wbc/state_estimation/kalman_filter.hpp"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& _A, // System dynamics.
    const Eigen::MatrixXd& _B, // Control.
    const Eigen::MatrixXd& _C, // Output.
    const Eigen::MatrixXd& _P, // Estimate error covariance.
    const Eigen::MatrixXd& _R, // Measurement noise covariance.
    const Eigen::MatrixXd& _Q  // Process noise covariance.
)
: A(_A)
, B(_B)
, C(_C)
, P(_P)
, R(_R)
, Q(_Q)
, P0(_P)
, I(A.rows(), A.rows())
, x_hat(A.rows())
{
  I.setIdentity();
  x_hat.setZero();
}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::init() {

  P = P0;
  x_hat.setZero();
}

void KalmanFilter::init(const Eigen::VectorXd& x) {

  init();
  x_hat = x;
}

void KalmanFilter::predict(const Eigen::VectorXd& u) {

  x_hat = A * x_hat + B * u;
  P = A*P*A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

	K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat += K * (y - C*x_hat);
	P = (I - K*C)*P;
	P = 0.5 * (P + P.transpose()); // keep P symmetric against round-off
}

Eigen::VectorXd KalmanFilter::get_state() const {
  return x_hat;
}

void KalmanFilter::set_Q(const Eigen::MatrixXd& _Q) {
  Q = _Q;
}

void KalmanFilter::set_R(const Eigen::MatrixXd& _R) {
  R = _R;
}

void KalmanFilter::set_state(const Eigen::VectorXd& x) {
  x_hat = x;
}

const Eigen::MatrixXd& KalmanFilter::get_P() const {
  return P;
}
