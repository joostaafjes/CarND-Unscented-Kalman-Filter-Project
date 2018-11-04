//
// Created by Joost Aafjes on 02/11/2018.
//

#include <iostream>
#include "radar_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

RadarKalmanFilter::RadarKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type)
    : UnscentedKalmanFilter(kalman_filter_state, supported_sensor_type) {
}

/**
  * Initialize the state x_ with the first measurement.
*/
bool RadarKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalman_filter_state_->is_initialized_) {
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */
    double radius = measurement_pack.raw_measurements_[0];
    double angle = measurement_pack.raw_measurements_[1];
    double px = cos(angle) * radius;
    double py = sin(angle) * radius;
    kalman_filter_state_->x_ << px, py, 0, 0, 0;

    // done initializing, no need to predict or update
    kalman_filter_state_->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 */
void RadarKalmanFilter::Update(const VectorXd &z) {
  /**

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // transform sigma points into measurement space
  for (int col = 0; col < no_of_colums_; col++) {
    double p_x = Xsig_pred_(0, col);
    double p_y = Xsig_pred_(1, col);
    double v = Xsig_pred_(2, col);
    double yaw = Xsig_pred_(3, col);
    double yaw_dot = Xsig_pred_(4, col);

    double rho = sqrt(p_x * p_x + p_y * p_y);
    double phi = atan2(p_y, p_x);
    double row_dot = (p_x * v * cos(yaw) + p_y * v * sin(yaw)) / rho;

    Zsig_.col(col) << rho, phi, row_dot;
  }

  // calculate mean predicted measurement
  z_pred_ = Zsig_ * weights_;

  // calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  S_.setZero(n_z_, n_z_);
  for (int col = 0; col < no_of_colums_; col++) {
    // predict state covariance matrix
    S_ << S_ + weights_(col) * (Zsig_.col(col) - z_pred_) * ((Zsig_.col(col) - z_pred_).transpose());
  }
  S_ = S_ + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  // calculate cross correlation matrix
  Tc.setZero();
  for (int col = 0; col < no_of_colums_; col++) {
    Tc = Tc + weights_(col) * ((Xsig_pred_.col(col) - kalman_filter_state_->x_) * ((Zsig_.col(col) - z_pred_).transpose()));
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * (S_.inverse());

  // update state mean
  kalman_filter_state_->x_ = kalman_filter_state_->x_ + K * (z - z_pred_);

  // update covariance matrix
  kalman_filter_state_->P_ = kalman_filter_state_->P_ - K * S_ * K.transpose();
}
