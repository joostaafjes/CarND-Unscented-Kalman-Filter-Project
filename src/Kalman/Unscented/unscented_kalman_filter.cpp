//
// Created by Joost Aafjes on 02/11/2018.
//

#include <iostream>
#include "unscented_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

UnscentedKalmanFilter::UnscentedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type)
    : VirtualKalmanFilter(kalman_filter_state, supported_sensor_type) {
}

/*
 * Initialize the state x_ with the first measurement.
 */
bool UnscentedKalmanFilter::Init(const MeasurementPackage &measurement_pack) {

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z_, no_of_colums_);

  //mean predicted measurement
  z_pred_ = VectorXd(n_z_);

  //measurement covariance matrix S
  S_ = MatrixXd(n_z_, n_z_);

  ///* Weights of sigma points
//  weights_ = ;

  ///* State dimension
  n_x_ = 5;

  n_z_ = 3;

  ///* Augmented state dimension
  n_aug_ = n_x_ + 2;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  no_of_colums_ = 2 * n_aug_ + 1;

  if (!kalman_filter_state_->is_initialized_) {
    kalman_filter_state_->x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    // done initializing, no need to predict or update
    kalman_filter_state_->Init(measurement_pack.timestamp_);
    return true;
  }

  return false;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UnscentedKalmanFilter::Predict() {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  float dt = kalman_filter_state_->dt;

  /*******************************************************************************
  * Student part begin (17: create augmented mean state, augmented covariance matrix and augmented signma points)
  ******************************************************************************/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, no_of_colums_);

  // create augmented mean state
  x_aug << kalman_filter_state_->x_, 0, 0;
  Xsig_aug.col(0) << x_aug;

  // create augmented covariance matrix
  P_aug.setZero(7, 7);
  P_aug.topLeftCorner(n_x_, n_x_) << kalman_filter_state_->P_;
  P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) << std_a_ * std_a_, 0,
      0, std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  for (int col = 0; col < n_aug_; col++)
  {
    Xsig_aug.col(col + 1)         = x_aug + sqrt(lambda_ + n_aug_) * A.col(col);
    Xsig_aug.col(col + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(col);
  }

/*******************************************************************************
 * Student part end
 ******************************************************************************/


/*******************************************************************************
 * Student part begin (20: sigma point prediction)
 ******************************************************************************/

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, no_of_colums_);

  Xsig_pred.setZero(n_x_, 2 * n_aug_ + 1);
  Xsig_pred.col(0) = Xsig_aug.col(0).head(5);
  for (int col = 0; col <= 2 * n_aug_; col++) {
    double p_x = Xsig_aug(0, col);
    double p_y = Xsig_aug(1, col);
    double v = Xsig_aug(2, col);
    double yaw = Xsig_aug(3, col);
    double yaw_dot = Xsig_aug(4, col);
    double nu_a = Xsig_aug(5, col);
    double nu_yaw_dotdot = Xsig_aug(6, col);
    VectorXd x = Xsig_aug.col(col).head(5);

    // predict sigma points
    VectorXd x_pred = VectorXd(5);
    if (yaw_dot != 0) {
      x_pred << v / yaw_dot * (sin(yaw + yaw_dot * dt) - sin(yaw)),
          v / yaw_dot * (-cos(yaw + yaw_dot * dt) + cos(yaw)),
          0, yaw_dot * dt, 0;
    } else {
      x_pred << v * cos(yaw) * dt, v * sin(yaw) * dt, 0, 0, 0;
    }
    VectorXd x_noise = VectorXd(5);
    x_noise << 0.5 * dt * dt * cos(yaw) * nu_a,
        0.5 * dt * dt * sin(yaw) * nu_a,
        dt * nu_a,
        0.5 * dt * dt * nu_yaw_dotdot,
        dt * nu_yaw_dotdot;

    VectorXd x_k1 = x + x_pred + x_noise;

    // avoid division by zero

    // write predicted sigma points into right column
    Xsig_pred.col(col) = x_k1;
  }

  /* udacity solution below:
  //predict sigma points
  for (int i = 0; i< 2*n_aug+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
   */

/*******************************************************************************
 * Student part end
 ******************************************************************************/



/*******************************************************************************
 * Student part begin (23: predict mean and covariance)
 ******************************************************************************/

  // set weights
  weights_.setConstant(no_of_colums_, 1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  std::cout << "Weights:" << std::endl;
  std::cout << weights_ << std::endl;

  // predict state mean
  kalman_filter_state_->x_ = Xsig_pred * weights_;
  std::cout << "x:" << std::endl;
  std::cout << kalman_filter_state_->x_ << std::endl;

  kalman_filter_state_->P_.setZero(n_x_, n_x_);
  std::cout << kalman_filter_state_->P_ << std::endl;
  for (int col = 0; col < no_of_colums_; col++) {
    // predict state covariance matrix
    std::cout << "col" << std::endl;
    std::cout << Xsig_pred.col(col) - kalman_filter_state_->x_ << std::endl;
    std::cout << "trn" << std::endl;
    std::cout << (Xsig_pred.col(col) - kalman_filter_state_->x_).transpose() << std::endl;
    std::cout << "wth" << std::endl;
    std::cout << (Xsig_pred.col(col) - kalman_filter_state_->x_) * ((Xsig_pred.col(col) - kalman_filter_state_->x_).transpose()) << std::endl;
    std::cout << "all" << std::endl;
    std::cout << weights_(col) * (Xsig_pred.col(col) - kalman_filter_state_->x_) * ((Xsig_pred.col(col) - kalman_filter_state_->x_).transpose()) << std::endl;
    kalman_filter_state_->P_ << kalman_filter_state_->P_ + weights_(col) * (Xsig_pred.col(col) - kalman_filter_state_->x_) * ((Xsig_pred.col(col) - kalman_filter_state_->x_).transpose());
    std::cout << kalman_filter_state_->P_ << std::endl;
  }


  /*
   * Udacity solution:
   *
  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    x = x+ weights(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }

   */


/*******************************************************************************
 * Student part end
 ******************************************************************************/


/*******************************************************************************
 * Student part begin (26: predict radar measurement)
 ******************************************************************************/

  // transform sigma points into measurement space
  for (int col = 0; col < no_of_colums_; col++) {
    double p_x = Xsig_pred(0, col);
    double p_y = Xsig_pred(1, col);
    double v = Xsig_pred(2, col);
    double yaw = Xsig_pred(3, col);
    double yaw_dot = Xsig_pred(4, col);

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
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

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





}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 */
void UnscentedKalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */



/*******************************************************************************
 * Student part begin (29: ukf update)
 ******************************************************************************/

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  // calculate cross correlation matrix
  Tc.setZero();
  for (int col = 0; col < no_of_colums_; col++) {
    Tc = Tc + weights_(col) * ((Xsig_pred_.col(col) - kalman_filter_state_->x_) * ((Zsig_.col(col) - z_pred_).transpose()));
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * (S_.inverse());

  // update state mean
  kalman_filter_state_->x_ = kalman_filter_state_->x_ + K * (z - z_pred_);

  // update covariance matrix
  kalman_filter_state_->P_ = kalman_filter_state_->P_ - K * S_ * K.transpose();

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
    VectorXd x_diff = Xsig_pred.col(i) - x;
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
