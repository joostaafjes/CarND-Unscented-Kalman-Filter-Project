#include <iostream>
#include "unscented_kalman_filter.h"
#include "../tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

UnscentedKalmanFilter::UnscentedKalmanFilter(KalmanFilterState *kalman_filter_state, SensorType supported_sensor_type) {
  Q_ = MatrixXd(4, 4);

  // Set the acceleration noise components
  noise_ax_ = 9;
  noise_ay_ = 9;

  // set state pointer
  kalman_filter_state_ = kalman_filter_state;

  // set supported sensor type
  supported_sensor_type_ = supported_sensor_type;

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

  ///* State dimension
  n_x_ = 5;

  n_z_ = 3;

  ///* Augmented state dimension
  n_aug_ = n_x_ + 2;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  no_of_colums_ = 2 * n_aug_ + 1;

  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z_, no_of_colums_);

  //mean predicted measurement
  z_pred_ = VectorXd(n_z_);

  //measurement covariance matrix S
  S_ = MatrixXd(n_z_, n_z_);

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, no_of_colums_);

  // set weights of sigma points
  weights_.setConstant(no_of_colums_, 1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
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
  for (int col = 0; col < n_aug_; col++) {
    Xsig_aug.col(col + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(col);
    Xsig_aug.col(col + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(col);
  }

//  std::cout << "Generated sigma points" << std::endl;
//  std::cout << Xsig_aug << std::endl;

/*******************************************************************************
 * Student part end
 ******************************************************************************/


/*******************************************************************************
 * Student part begin (20: sigma point prediction)
 ******************************************************************************/

  Xsig_pred_.setZero(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.col(0) = Xsig_aug.col(0).head(5);
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
    Xsig_pred_.col(col) = x_k1;
  }

//  std::cout << "Predicted sigma points" << std::endl;
//  std::cout << Xsig_pred_ << std::endl;

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
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
   */

/*******************************************************************************
 * Student part end
 ******************************************************************************/



/*******************************************************************************
 * Student part begin (23: predict mean and covariance)
 ******************************************************************************/

//  std::cout << "x_(before):" << std::endl;
//  std::cout << kalman_filter_state_->x_ << std::endl;

//  std::cout << "xsig * weights:" << std::endl;
//  std::cout << Xsig_pred_ * weights_ << std::endl;

  // predict state mean
  kalman_filter_state_->x_ = Xsig_pred_ * weights_;

  kalman_filter_state_->P_.setZero(n_x_, n_x_);
  for (int col = 0; col < no_of_colums_; col++) {
    // predict state covariance matrix
    kalman_filter_state_->P_ << kalman_filter_state_->P_
        + weights_(col) * (Xsig_pred_.col(col) - kalman_filter_state_->x_)
            * ((Xsig_pred_.col(col) - kalman_filter_state_->x_).transpose());
  }

//  std::cout << "weights_:" << std::endl;
//  std::cout << weights_ << std::endl;

  std::cout << "x_:" << std::endl;
  std::cout << kalman_filter_state_->x_ << std::endl;

  std::cout << "P_:" << std::endl;
  std::cout << kalman_filter_state_->P_ << std::endl;

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
    x = x+ weights(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }

   */


/*******************************************************************************
 * Student part end
 ******************************************************************************/
}

