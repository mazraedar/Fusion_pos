#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "kalman.hpp"

int fusion(double dt, const Eigen::VectorXd& y,const Eigen::VectorXd x0,const Eigen::MatrixXd& C) {

  int n = 3; // Number of states
  int m = 6; // Number of measurements

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Free object
  A << 1, 0, 0,
	0, 1, 0,
	0, 0, 1;

  // Covariance matrices
  Q << 1, 0, 0,
	0, 1, 0,
	0, 0, 1;
  R << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  P << 1, 0, 0,
	0, 1, 0,
	0, 0, 1;

  // Construct the filter
  KalmanFilter kf(dt, A, C, Q, R, P);
  kf.init(dt, x0);
  kf.update(y);
}
