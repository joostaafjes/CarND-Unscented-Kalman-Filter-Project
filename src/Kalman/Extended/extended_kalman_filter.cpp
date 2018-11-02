//
// Created by Joost Aafjes on 22/10/2018.
//

#include "extended_kalman_filter.h"
#include "../../tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ExtendedKalmanFilter::ExtendedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type)
    : VirtualKalmanFilter(kalman_filter_state, supported_sensor_type) {
  /*
   * measurement covariance matrix - radar
   */
  R_ = MatrixXd(3, 3);
  R_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
}

/**
  * Initialize the state x_ with the first measurement.
*/
bool ExtendedKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalman_filter_state_->is_initialized_) {
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */
    double radius = measurement_pack.raw_measurements_[0];
    double angle = measurement_pack.raw_measurements_[1];
    double px = cos(angle) * radius;
    double py = sin(angle) * radius;
    kalman_filter_state_->x_ << px, py, 0, 0;

    // done initializing, no need to predict or update
    kalman_filter_state_->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

void ExtendedKalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
    */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(kalman_filter_state_->x_);

  VectorXd y = z - ConvertCartesianToPolar(kalman_filter_state_->x_);

  NormalizeAngle(&y(1));

  MatrixXd Hj_t = Hj.transpose();
  MatrixXd S = Hj * kalman_filter_state_->P_ * Hj_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = kalman_filter_state_->P_ * Hj_t;
  MatrixXd K = PHt * Si;

  // new estimate
  kalman_filter_state_->x_ = kalman_filter_state_->x_ + (K * y);
  long x_size = kalman_filter_state_->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  kalman_filter_state_->P_ = (I - K * Hj) * kalman_filter_state_->P_;
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

void ExtendedKalmanFilter::NormalizeAngle(double *angle) {
  while (*angle > M_PI) {
    std::cout << "phi updated(-):" << *angle << std::endl;
    *angle -= 2 * M_PI;
  }
  while (*angle < -M_PI) {
    std::cout << "phi updated(+):" << *angle << std::endl;
    *angle += 2 * M_PI;
  }
}
