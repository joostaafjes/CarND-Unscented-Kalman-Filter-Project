#ifndef FUSION_EKF_H
#define FUSION_EKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <list>
#include "Kalman/Standard/standard_kalman_filter.h"
#include "Kalman/Extended/extended_kalman_filter.h"
#include "tools.h"
#include "Kalman/kalman_filter_state.h"

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

  KalmanFilterState *kalman_filter_state_;

private:
  // tool object used to compute Jacobian and RMSE
  Tools tools;

  std::list<VirtualKalmanFilter*> kalman_filter_list_;

};

#endif // FUSION_EKF_H
