//
// Created by Joost Aafjes on 22/10/2018.
//

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H
#include <iostream>
#include "../Eigen/Dense"
#include "../measurement_package.h"
#include "standard_kalman_filter.h"
#include "kalman_filter_state.h"
#include "virtual_kalman_filter.h"

class ExtendedKalmanFilter: public VirtualKalmanFilter {
 public:
  ExtendedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type);

  bool Init(const MeasurementPackage &measurement_pack);
  void Update(const Eigen::VectorXd &z);
 private:
  Eigen::VectorXd ConvertCartesianToPolar(Eigen::VectorXd x);
  void NormalizeAngle(double* angle);
};

#endif //EXTENDED_KALMAN_FILTER_H
