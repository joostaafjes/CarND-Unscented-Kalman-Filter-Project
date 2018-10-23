#include <iostream>
#include "standard_kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

StandardKalmanFilter::StandardKalmanFilter(KalmanFilterState *pKalmanFilterState, SensorType supported_sensortype)
    : VirtualKalmanFilter(pKalmanFilterState, supported_sensortype) {
  // initializing matrices
  R_ = MatrixXd(2, 2);

  // measurement covariance matrix - laser
  R_ << 0.0225, 0,
      0, 0.0225;
}

StandardKalmanFilter::~StandardKalmanFilter() {}

/*
 * Initialize the state x_ with the first measurement.
 */
bool StandardKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalmanFilterState->is_initialized_) {
    kalmanFilterState->x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    // done initializing, no need to predict or update
    kalmanFilterState->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

void StandardKalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * kalmanFilterState->x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * kalmanFilterState->P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = kalmanFilterState->P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  kalmanFilterState->x_ = kalmanFilterState->x_ + (K * y);
  long x_size = kalmanFilterState->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  kalmanFilterState->P_ = (I - K * H_) * kalmanFilterState->P_;
}

