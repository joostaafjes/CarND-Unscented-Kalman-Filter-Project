#ifndef VIRTUAL_KALMAN_FILTER_H_
#define VIRTUAL_KALMAN_FILTER_H_
#include "../Eigen/Dense"
#include "../measurement_package.h"
#include "kalman_filter_state.h"

class VirtualKalmanFilter {
 public:

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_; // only for lidar

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  VirtualKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type);

  /**
   * Destructor
   */
  virtual ~VirtualKalmanFilter() = default;

  /**
   * Initialize the state x_ with the first measurement.
   */
  virtual bool Init(const MeasurementPackage &measurement_pack) = 0;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  virtual void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void Update(const Eigen::VectorXd &z) = 0;

  SensorType supported_sensor_type_;

  KalmanFilterState *kalman_filter_state_;
 private:
  float noise_ax_;
  float noise_ay_;
};

#endif /* VIRTUAL_KALMAN_FILTER_H_ */
