#include "kalman_filter.h"

#include <math.h>

#define PI 3.14159265  // PI number definition

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// Predict state
void KalmanFilter::Predict(float dt) {
  // Update the state transition matrix F according to the new elapsed time
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Aux variables to process covariance matrix Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Calculate the process covariance matrix Q
  Q_ = MatrixXd(4, 4);
  Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0,
        0, dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_,
        dt_3 / 2 * noise_ax_, 0, dt_2 * noise_ax_, 0,
        0, dt_3 / 2 * noise_ay_, 0, dt_2 * noise_ay_;

  // Predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

// Update measurement using kalman filter equations for Lidar sensor
void KalmanFilter::Update(const VectorXd &z) {
  // Calculate the error between measurement and prediction 
  VectorXd y = z - H_ * x_;

  // Calculate S and K
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

// Update measurement using extended kalman filter equations for radar sensor
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Calculate h(x')
  VectorXd hx = VectorXd(3);
  float c1 = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float c2 = std::atan2(x_(1), x_(0));
  float c3 = 0;

  // Avoid division by zero
  if (c1 != 0) {
    c3 = (x_(0) * x_(2) + x_(1) * x_(3)) / c1;
  }
  hx << c1, c2, c3;

  // Calculate the error between measurement and prediction
  VectorXd y = z - hx;

  // Assure that y(1) will be between -PI and PI
  while (y(1) > PI || y(1) < -PI) {
    if (y(1) > 0) {
      y(1) = y(1) - 2 * PI;
    } else {
      y(1) = y(1) + 2 * PI;
    }
  }

  // Calculate S and K
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
