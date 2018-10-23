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
  ExtendedKalmanFilter(KalmanFilterState *pKalmanFilterState, SensorType supportedSensorType);

  bool Init(const MeasurementPackage &measurementPack);
  void Update(const Eigen::VectorXd &z);
 private:
  Eigen::VectorXd ConvertCartesianToPolar(Eigen::VectorXd x);
  void NormalizeAngle(double* pangle);
};

#endif //EXTENDED_KALMAN_FILTER_H
