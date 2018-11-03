#ifndef VIRTUAL_KALMAN_FILTER_H_
#define VIRTUAL_KALMAN_FILTER_H_
#include "../Eigen/Dense"
#include "../measurement_package.h"
#include "kalman_filter_state.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter {
 public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig_;

  //mean predicted measurement
  VectorXd z_pred_;

  //measurement covariance matrix S
  MatrixXd S_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  int n_z_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  int no_of_colums_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  UnscentedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type);

  /**
   * Destructor
   */
  virtual ~UnscentedKalmanFilter() = default;

  /**
   * Initialize the state x_ with the first measurement.
   */
  virtual bool Init(const MeasurementPackage &measurement_pack) = 0;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

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
