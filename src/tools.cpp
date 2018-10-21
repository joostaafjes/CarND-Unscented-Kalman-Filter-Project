#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() == 0) {
    cout << "Error: Estimations array is empty";
    return rmse;
  }

  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()) {
    cout << "Error: Estimations and ground truth array not equal";
    return rmse;
  }

  //accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    VectorXd diff_x_diff = diff.array() * diff.array();
    rmse += diff_x_diff;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if (px == 0 && py == 0) {
    cout << "Error: px and py are 0 -> division by zero!";
    return Hj;
  }

  // compute the Jacobian matrix
  double sum_px2_py2 = px * px + py * py;
  double sqrt_sum_px2_py2 = sqrt(sum_px2_py2);
  double sum_px2_py2_3_div_2 = pow(sum_px2_py2, 1.5);

  Hj << px / sqrt_sum_px2_py2, py / sqrt_sum_px2_py2, 0, 0,
      - py / sum_px2_py2, px / sum_px2_py2, 0, 0,
      py * (vx * py - vy * px) / sum_px2_py2_3_div_2, px * (vy * px - vx * py) / sum_px2_py2_3_div_2,
//      py * (vx * py - vy * px) / sum_px2_py2_3_div_2, px * (vx * py - vy * px) / sum_px2_py2_3_div_2,
      px / sqrt_sum_px2_py2, py / sqrt_sum_px2_py2;

  return Hj;
}
