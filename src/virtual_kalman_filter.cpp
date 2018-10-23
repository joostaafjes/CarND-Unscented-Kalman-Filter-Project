#include <iostream>
#include "standard_kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

VirtualKalmanFilter::VirtualKalmanFilter(KalmanFilterState *pKalmanFilterState, SensorType supported_sensortype) {
  // initializing matrices
  H_ = MatrixXd(2, 4);

  // measurement matrix
  H_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Q_ = MatrixXd(4, 4);

  // set state pointer
  kalmanFilterState = pKalmanFilterState;

  supportedSensorType = supported_sensortype;
}

VirtualKalmanFilter::~VirtualKalmanFilter() {}

bool VirtualKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalmanFilterState->is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

      /**
      Initialize state.
      */
      VectorXd x = VectorXd(4);
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      kalmanFilterState->x_ = x;
//      ekf_.setState(x);

    // done initializing, no need to predict or update
    kalmanFilterState->previous_timestamp_ = measurement_pack.timestamp_;
    kalmanFilterState->is_initialized_ = true;
    return true;
  }

  return false;
}

void VirtualKalmanFilter::Predict() {
  float dt = kalmanFilterState->dt;
  // 1. Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // set the acceleration noise components
  float noise_ax_ = 9; // was 5
  float noise_ay_ = 9; // was 5

  // 2. Set the process covariance matrix Q
  Q_ << pow(dt, 4) / 4 * noise_ax_, 0, pow(dt, 3) / 2 * noise_ax_, 0,
      0, pow(dt, 4) / 4 * noise_ay_, 0, pow(dt, 3) / 2 * noise_ay_,
      pow(dt, 3) / 2 * noise_ax_, 0, pow(dt, 2) * noise_ax_, 0,
      0, pow(dt, 3) / 2 * noise_ay_, 0, pow(dt, 2) * noise_ay_;

  /**
    * predict the state
  */
  kalmanFilterState->x_ = F_ * kalmanFilterState->x_;
  MatrixXd Ft = F_.transpose();
  kalmanFilterState->P_ = F_ * kalmanFilterState->P_ * Ft + Q_;
}

