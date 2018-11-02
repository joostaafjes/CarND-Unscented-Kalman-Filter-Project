//
// Created by Joost Aafjes on 02/11/2018.
//

#ifndef UNSCENTEDKF_UNSCENTED_KALMAN_FILTER_H
#define UNSCENTEDKF_UNSCENTED_KALMAN_FILTER_H

#include "../../Eigen/Dense"
#include "../../measurement_package.h"
#include "../kalman_filter_state.h"
#include "../virtual_kalman_filter.h"

class UnscentedKalmanFilter: public VirtualKalmanFilter {
 public:
  /**
   * Constructor
   */
  UnscentedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type);

  bool Init(const MeasurementPackage &measurementPack) override;

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z) override;

};

#endif //UNSCENTEDKF_UNSCENTED_KALMAN_FILTER_H
