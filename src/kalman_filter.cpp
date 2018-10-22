#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter(KalmanFilterState *pKalmanFilterState) {
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_ = MatrixXd(2, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // measurement matrix
  H_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  // set state pointer
  kalmanFilterState = pKalmanFilterState;
}

KalmanFilter::~KalmanFilter() {}

bool KalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalmanFilterState->is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double radius = measurement_pack.raw_measurements_[0];
      double angle = measurement_pack.raw_measurements_[1];
      double px = sin(angle) * radius;
      double py = cos(angle) * radius;
      VectorXd x = VectorXd(4);
      x << px, py, 0, 0;
      kalmanFilterState->x_ = x;
//      ekf_.setState(x);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      VectorXd x = VectorXd(4);
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      kalmanFilterState->x_ = x;
//      ekf_.setState(x);
    }

    // done initializing, no need to predict or update
    kalmanFilterState->previous_timestamp_ = measurement_pack.timestamp_;
    kalmanFilterState->is_initialized_ = true;
    return true;
  }

  return false;
}

void KalmanFilter::Predict() {
  float dt = kalmanFilterState->dt;
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
  kalmanFilterState->x_ = F_ * kalmanFilterState->x_;
  MatrixXd Ft = F_.transpose();
  kalmanFilterState->P_ = F_ * kalmanFilterState->P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * kalmanFilterState->x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * kalmanFilterState->P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = kalmanFilterState->P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  kalmanFilterState->x_ = kalmanFilterState->x_ + (K * y);
  long x_size = kalmanFilterState->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  kalmanFilterState->P_ = (I - K * H_) * kalmanFilterState->P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(kalmanFilterState->x_);

  VectorXd y = z - ConvertCartesianToPolar(kalmanFilterState->x_);

  NormalizeAngle(&y(1));

  MatrixXd Hj_t = Hj.transpose();
  MatrixXd S = Hj * kalmanFilterState->P_ * Hj_t + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = kalmanFilterState->P_ * Hj_t;
  MatrixXd K = PHt * Si;

  // new estimate
  kalmanFilterState->x_ = kalmanFilterState->x_ + (K * y);
  long x_size = kalmanFilterState->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  kalmanFilterState->P_ = (I - K * Hj) * kalmanFilterState->P_;
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
