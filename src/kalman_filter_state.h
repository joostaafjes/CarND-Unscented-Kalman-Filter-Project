//
// Created by Joost Aafjes on 22/10/2018.
//

#ifndef EXTENDEDKF_KALMAN_FILTER_STATE_H
#define EXTENDEDKF_KALMAN_FILTER_STATE_H

#include "Eigen/Dense"

class KalmanFilterState {
 public:
  // constructor
  KalmanFilterState();

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  float dt;

  void UpdateDateTime(long long timestamp);
};

#endif //EXTENDEDKF_KALMAN_FILTER_STATE_H
