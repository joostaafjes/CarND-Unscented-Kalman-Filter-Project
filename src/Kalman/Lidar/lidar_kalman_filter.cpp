#include <iostream>
#include "lidar_kalman_filter.h"
#include "../../tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

LidarKalmanFilter::LidarKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type)
    : UnscentedKalmanFilter(kalman_filter_state, supported_sensor_type) {
  // initializing matrices
  R_ = MatrixXd(2, 2);

  // measurement covariance matrix - laser
  R_ << 0.0225, 0,
      0, 0.0225;

  // measurement matrix
  H_ = MatrixXd(2, 5);
  H_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;

}

/*
 * Initialize the state x_ with the first measurement.
 */
bool LidarKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalman_filter_state_->is_initialized_) {
    kalman_filter_state_->x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;

    // done initializing, no need to predict or update
    kalman_filter_state_->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

void LidarKalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * kalman_filter_state_->x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * kalman_filter_state_->P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = kalman_filter_state_->P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  kalman_filter_state_->x_ = kalman_filter_state_->x_ + (K * y);
  long x_size = kalman_filter_state_->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  kalman_filter_state_->P_ = (I - K * H_) * kalman_filter_state_->P_;
}

