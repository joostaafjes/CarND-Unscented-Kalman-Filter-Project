#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "../../Eigen/Dense"
#include "../../measurement_package.h"
#include "../kalman_filter_state.h"
#include "../unscented_kalman_filter.h"

class LidarKalmanFilter: public UnscentedKalmanFilter {
public:
  /**
   * Constructor
   */
  LidarKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type);

  bool Init(const MeasurementPackage &measurementPack) override;

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z) override;
 private:
  // measurement matrix
  Eigen::MatrixXd H_; // only for lidar

};

#endif /* KALMAN_FILTER_H_ */
