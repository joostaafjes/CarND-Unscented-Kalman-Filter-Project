//
// Created by Joost Aafjes on 22/10/2018.
//

#include "kalman_filter_state.h"

KalmanFilterState::KalmanFilterState() {

  // state covariance matrix P
  P_ = Eigen::MatrixXd(5, 5);
//  P_ << 1, 0, 0, 0,
//      0, 1, 0, 0,
//      0, 0, 1000, 0,
//      0, 0, 0, 1000;
  P_ << Eigen::MatrixXd::Identity(5, 5);

  x_ = Eigen::VectorXd(5);

  is_initialized_ = false;

  previous_timestamp_ = 0;
}

void KalmanFilterState::Init(long long timestamp) {
  previous_timestamp_ = timestamp;
  is_initialized_ = true;
}

void KalmanFilterState::UpdateDateTime(long long timestamp) {
  dt = (timestamp - previous_timestamp_) / 1000000.0;    // dt - expressed in seconds
  previous_timestamp_ = timestamp;
}
