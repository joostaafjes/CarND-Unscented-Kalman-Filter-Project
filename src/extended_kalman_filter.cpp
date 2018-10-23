//
// Created by Joost Aafjes on 22/10/2018.
//

#include "extended_kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ExtendedKalmanFilter::ExtendedKalmanFilter(KalmanFilterState *pKalmanFilterState, SensorType supported_sensor_type)
  : VirtualKalmanFilter(pKalmanFilterState, supported_sensor_type) {
  // initializing matrices
  R_ = MatrixXd(3, 3);

  // measurement covariance matrix - radar
  R_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
}

bool ExtendedKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalmanFilterState->is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement

      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double radius = measurement_pack.raw_measurements_[0];
      double angle = measurement_pack.raw_measurements_[1];
      double px = sin(angle) * radius;
      double py = cos(angle) * radius;
      kalmanFilterState->x_ << px, py, 0, 0;

    // done initializing, no need to predict or update
    kalmanFilterState->previous_timestamp_ = measurement_pack.timestamp_;
    kalmanFilterState->is_initialized_ = true;
    return true;
  }

  return false;
}

void ExtendedKalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(kalmanFilterState->x_);

  VectorXd y = z - ConvertCartesianToPolar(kalmanFilterState->x_);

  NormalizeAngle(&y(1));

  MatrixXd Hj_t = Hj.transpose();
  MatrixXd S = Hj * kalmanFilterState->P_ * Hj_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = kalmanFilterState->P_ * Hj_t;
  MatrixXd K = PHt * Si;

  // new estimate
  kalmanFilterState->x_ = kalmanFilterState->x_ + (K * y);
  long x_size = kalmanFilterState->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  kalmanFilterState->P_ = (I - K * Hj) * kalmanFilterState->P_;
}

Eigen::VectorXd ExtendedKalmanFilter::ConvertCartesianToPolar(Eigen::VectorXd x) {
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

void ExtendedKalmanFilter::NormalizeAngle(double* pangle) {
  while (*pangle > M_PI) {
    std::cout << "phi updated(-):" << *pangle << std::endl;
    *pangle -= 2*M_PI;
  }
  while (*pangle < -M_PI) {
    std::cout << "phi updated(+):" << *pangle << std::endl;
    *pangle += 2*M_PI;
  }
}
