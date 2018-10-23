#ifndef FUSION_EKF_H
#define FUSION_EKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <list>
#include "standard_kalman_filter.h"
#include "extended_kalman_filter.h"
#include "tools.h"
#include "kalman_filter_state.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  StandardKalmanFilter *pEkf_;

  KalmanFilterState *pKalmanFilterState;

private:
  // tool object used to compute Jacobian and RMSE
  Tools tools;

  std::list<VirtualKalmanFilter*> kalmanFilterList;

};

#endif // FUSION_EKF_H
