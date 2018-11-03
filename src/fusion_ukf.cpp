#include "fusion_ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionUKF::FusionUKF() {
  kalman_filter_state_ = new KalmanFilterState();

  UnscentedKalmanFilter* standard_kalman_filter = new LidarKalmanFilter(kalman_filter_state_, SensorType::LASER);
  kalman_filter_list_.push_front(standard_kalman_filter);

  UnscentedKalmanFilter* unscented_kalman_filter = new RadarKalmanFilter(kalman_filter_state_, SensorType::RADAR);
  kalman_filter_list_.push_front(unscented_kalman_filter);
}

/**
* Destructor.
*/
FusionUKF::~FusionUKF() {
  delete kalman_filter_state_;
}

void FusionUKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  for (auto filter = kalman_filter_list_.begin(); filter != kalman_filter_list_.end(); ++filter) {
    /*
     * Match sensor type
     */
    if ((*filter)->supported_sensor_type_ == measurement_pack.sensor_type_) {
      /*****************************************************************************
       *  Initialization
       ****************************************************************************/
      if ((*filter)->Init(measurement_pack)) {
        return;
      }

      /*
       * Compute the time elapsed between the current and previous measurements
       */
      kalman_filter_state_->UpdateDateTime(measurement_pack.timestamp_);

      /*****************************************************************************
       *  Prediction:
       *
       * Update the state transition matrix F according to the new elapsed time.
         - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax_ = 9 and noise_ay_ = 9 for your Q matrix.
       ****************************************************************************/
      (*filter)->Predict();

      /*****************************************************************************
       *  Update
       *
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
       ****************************************************************************/
      (*filter)->Update(measurement_pack.raw_measurements_);

      // print the output
//  cout << "x_ = " << ekf_.x_ << endl;
//  cout << "P_ = " << ekf_.P_ << endl;
    }
  }
}

