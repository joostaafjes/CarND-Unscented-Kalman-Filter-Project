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
  VirtualKalmanFilter* standardKalmanFilter = new StandardKalmanFilter(pKalmanFilterState, SensorType::LASER);
  VirtualKalmanFilter* extendedKalmanFilter = new ExtendedKalmanFilter(pKalmanFilterState, SensorType::RADAR);
  kalmanFilterList.push_front(standardKalmanFilter);
  kalmanFilterList.push_front(extendedKalmanFilter);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {
  delete pKalmanFilterState;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  for (auto filter = kalmanFilterList.begin(); filter != kalmanFilterList.end(); ++filter) {
    /*
     * Match sensor type
     */
    if ((*filter)->supportedSensorType == measurement_pack.sensor_type_) {
      /*****************************************************************************
       *  Initialization
       ****************************************************************************/
      if ((*filter)->Init(measurement_pack)) {
        return;
      }

      /*
       * Compute the time elapsed between the current and previous measurements
       */
      pKalmanFilterState->UpdateDateTime(measurement_pack.timestamp_);

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

