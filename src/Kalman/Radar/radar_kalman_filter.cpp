//
// Created by Joost Aafjes on 02/11/2018.
//

#include <iostream>
#include "radar_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

RadarKalmanFilter::RadarKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type)
    : UnscentedKalmanFilter(kalman_filter_state, supported_sensor_type) {

}

/**
  * Initialize the state x_ with the first measurement.
*/
bool RadarKalmanFilter::Init(const MeasurementPackage &measurement_pack) {
  if (!kalman_filter_state_->is_initialized_) {
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */
    double radius = measurement_pack.raw_measurements_[0];
    double angle = measurement_pack.raw_measurements_[1];
    double px = cos(angle) * radius;
    double py = sin(angle) * radius;
    kalman_filter_state_->x_ << px, py, 0, 0, 0;

    // done initializing, no need to predict or update
    kalman_filter_state_->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 */
void RadarKalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */


/*******************************************************************************
 * Student part begin (26: predict radar measurement)
 ******************************************************************************/

  // transform sigma points into measurement space
  for (int col = 0; col < no_of_colums_; col++) {
    double p_x = Xsig_pred_(0, col);
    double p_y = Xsig_pred_(1, col);
    double v = Xsig_pred_(2, col);
    double yaw = Xsig_pred_(3, col);
    double yaw_dot = Xsig_pred_(4, col);

    double rho = sqrt(p_x * p_x + p_y * p_y);
    double phi = atan2(p_y, p_x);
    double row_dot = (p_x * v * cos(yaw) + p_y * v * sin(yaw)) / rho;

    Zsig_.col(col) << rho, phi, row_dot;
  }

  // calculate mean predicted measurement
  z_pred_ = Zsig_ * weights_;

  // calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  S_.setZero(n_z_, n_z_);
  for (int col = 0; col < no_of_colums_; col++) {
    // predict state covariance matrix
    S_ << S_ + weights_(col) * (Zsig_.col(col) - z_pred_) * ((Zsig_.col(col) - z_pred_).transpose());
  }
  S_ = S_ + R;

  /*
   * Udacity solution:
   *  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;
   */

/*******************************************************************************
 * Student part end
 ******************************************************************************/

/*******************************************************************************
 * Student part begin (29: ukf update)
 ******************************************************************************/

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  // calculate cross correlation matrix
  Tc.setZero();
  for (int col = 0; col < no_of_colums_; col++) {
//    std::cout << Tc << std::endl;
//    std::cout << weights_(col) << std::endl;
//    std::cout << Xsig_pred_.col(col) << std::endl;
//    std::cout << kalman_filter_state_->x_ << std::endl;
//    std::cout << Zsig_.col(col) << std::endl;
//    std::cout << z_pred_ << std::endl;
    Tc = Tc + weights_(col) * ((Xsig_pred_.col(col) - kalman_filter_state_->x_) * ((Zsig_.col(col) - z_pred_).transpose()));
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * (S_.inverse());

  // update state mean
  kalman_filter_state_->x_ = kalman_filter_state_->x_ + K * (z - z_pred_);

  // update covariance matrix
  kalman_filter_state_->P_ = kalman_filter_state_->P_ - K * S_ * K.transpose();

  std::cout << "x_(update):" << std::endl;
  std::cout << kalman_filter_state_->x_ << std::endl;

  std::cout << "P_(update):" << std::endl;
  std::cout << kalman_filter_state_->P_ << std::endl;

  /*
   * Udacity solution:
   *
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();
   */

/*******************************************************************************
 * Student part end
 ******************************************************************************/

}
