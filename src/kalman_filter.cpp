#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::setState(VectorXd &x_in) {
  x_ = x_in;
}

void KalmanFilter::Init(MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_radar_in, Eigen::MatrixXd &R_lasar_in) {
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_radar_ = R_radar_in;
  R_lasar_ = R_lasar_in;
}

void KalmanFilter::Predict(float dt) {
  // 1. Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // set the acceleration noise components
  float noise_ax_ = 9; // was 5
  float noise_ay_ = 9; // was 5

  // 2. Set the process covariance matrix Q
  MatrixXd Q_lasar = MatrixXd(4, 4);
  Q_lasar << pow(dt, 4) / 4 * noise_ax_, 0, pow(dt, 3) / 2 * noise_ax_, 0,
      0, pow(dt, 4) / 4 * noise_ay_, 0, pow(dt, 3) / 2 * noise_ay_,
      pow(dt, 3) / 2 * noise_ax_, 0, pow(dt, 2) * noise_ax_, 0,
      0, pow(dt, 3) / 2 * noise_ay_, 0, pow(dt, 2) * noise_ay_;
  Q_ = Q_lasar;

  /**
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_lasar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);

  VectorXd y = z - ConvertCartesianToPolar(x_);

  NormalizeAngle(&y(1));

  MatrixXd Hj_t = Hj.transpose();
  MatrixXd S = Hj * P_ * Hj_t + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hj_t;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}

Eigen::VectorXd KalmanFilter::ConvertCartesianToPolar(Eigen::VectorXd x) {
  double p_x = x(0);
  double p_y = x(1);
  double rho = sqrt(p_x * p_x + p_y * p_y);
  double vx = x(2);
  double vy = x(3);

  double phi = atan2(p_y, p_x);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, (p_x * vx + p_y * vy) / rho;

  return z_pred;
}

void KalmanFilter::NormalizeAngle(double* pangle) {
  while (*pangle > M_PI) {
    std::cout << "phi updated(-):" << *pangle << std::endl;
    *pangle -= 2*M_PI;
  }
  while (*pangle < -M_PI) {
    std::cout << "phi updated(+):" << *pangle << std::endl;
    *pangle += 2*M_PI;
  }
}
