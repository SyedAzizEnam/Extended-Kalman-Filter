#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_laser_in,
                        MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;

  I = MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * (H_.transpose()) + R_laser_;
  MatrixXd K = P_ * (H_.transpose()) * (S.inverse());

  //new state
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const VectorXd &z_pred) {

  VectorXd y = z - z_pred;
  MatrixXd S = Hj_ * P_ * Hj_.transpose() + R_radar_;
  MatrixXd K = P_ * Hj_.transpose() * S.inverse();

  //new state
  x_ = x_ + K * y;
  P_ = (I - K * Hj_) * P_;
}
