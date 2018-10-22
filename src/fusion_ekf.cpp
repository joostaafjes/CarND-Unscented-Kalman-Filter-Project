#include "fusion_ekf.h"
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
FusionEKF::FusionEKF() {
  pKalmanFilterState = new KalmanFilterState();
  pEkf_ = new KalmanFilter(pKalmanFilterState);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {
  delete pKalmanFilterState;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
   if (pEkf_->Init(measurement_pack)) {
     return;
   }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax_ = 9 and noise_ay_ = 9 for your Q matrix.
   */
  // compute the time elapsed between the current and previous measurements
  pKalmanFilterState->UpdateDateTime(measurement_pack.timestamp_);

  pEkf_->Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    pEkf_->UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    pEkf_->Update(measurement_pack.raw_measurements_);
  }

  // print the output
//  cout << "x_ = " << ekf_.x_ << endl;
//  cout << "P_ = " << ekf_.P_ << endl;
}
