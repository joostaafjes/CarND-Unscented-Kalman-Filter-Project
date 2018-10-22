#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "measurement_package.h"
#include "kalman_filter_state.h"

class KalmanFilter {
public:

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_; // only for lidar

  // measurement covariance matrix
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd R_laser_;

  /**
   * Constructor
   */
  KalmanFilter();
  KalmanFilter(KalmanFilterState *pKalmanFilterState);

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  bool Init(const MeasurementPackage &measurementPack);

  /**
   * Init Initializes Kalman filter
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   */
//  void Init(Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
//      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_radar_in, Eigen::MatrixXd &R_lasar_in_);

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
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);
 private:
  Eigen::VectorXd ConvertCartesianToPolar(Eigen::VectorXd x);
  void NormalizeAngle(double* pangle);
  KalmanFilterState *kalmanFilterState;

};

#endif /* KALMAN_FILTER_H_ */
