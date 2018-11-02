//
// Created by Joost Aafjes on 02/11/2018.
//

#include "unscented_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

UnscentedKalmanFilter::UnscentedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type)
    : VirtualKalmanFilter(kalman_filter_state, supported_sensor_type) {
}

/*
 * Initialize the state x_ with the first measurement.
 */
bool UnscentedKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalman_filter_state_->is_initialized_) {
    kalman_filter_state_->x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    // done initializing, no need to predict or update
    kalman_filter_state_->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

void UnscentedKalmanFilter::Update(const VectorXd &z) {
}
