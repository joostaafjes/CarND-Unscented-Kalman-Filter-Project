#include <iostream>
#include "standard_kalman_filter.h"
#include "../tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

VirtualKalmanFilter::VirtualKalmanFilter(KalmanFilterState *pKalmanFilterState, SensorType supported_sensortype) {
  // measurement matrix
  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);

  // Set the acceleration noise components
  noise_ax_ = 9;
  noise_ay_ = 9;

  // set state pointer
  kalmanFilterState = pKalmanFilterState;

  // set supported sensor type
  supportedSensorType = supported_sensortype;
}

VirtualKalmanFilter::~VirtualKalmanFilter() {}

void VirtualKalmanFilter::Predict() {
  float dt = kalmanFilterState->dt;
  // Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Update the process covariance matrix Q
  Q_ << pow(dt, 4) / 4 * noise_ax_, 0, pow(dt, 3) / 2 * noise_ax_, 0,
      0, pow(dt, 4) / 4 * noise_ay_, 0, pow(dt, 3) / 2 * noise_ay_,
      pow(dt, 3) / 2 * noise_ax_, 0, pow(dt, 2) * noise_ax_, 0,
      0, pow(dt, 3) / 2 * noise_ay_, 0, pow(dt, 2) * noise_ay_;

  /*
   * Predict the state
   */
  kalmanFilterState->x_ = F_ * kalmanFilterState->x_;
  MatrixXd Ft = F_.transpose();
  kalmanFilterState->P_ = F_ * kalmanFilterState->P_ * Ft + Q_;
}

