#ifndef FUSION_EKF_H
#define FUSION_EKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <list>
#include "Kalman/unscented_kalman_filter.h"
#include "Kalman/Lidar/lidar_kalman_filter.h"
#include "Kalman/Radar/radar_kalman_filter.h"
#include "tools.h"
#include "Kalman/kalman_filter_state.h"

class FusionUKF {
public:
  /**
  * Constructor.
  */
  FusionUKF();

  /**
  * Destructor.
  */
  virtual ~FusionUKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  UnscentedKalmanFilter *pEkf_;

  KalmanFilterState *kalman_filter_state_;

private:
  // tool object used to compute Jacobian and RMSE
  Tools tools;

  std::list<UnscentedKalmanFilter*> kalman_filter_list_;

};

#endif // FUSION_EKF_H
