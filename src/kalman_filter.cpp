#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;

// Hazem
#include "tools.h"

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */

  cout << " ******************************************* PREDICT ******************************************** " << std::endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  cout << "(Debug) - x_= " << x_ << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  cout << " ******************************************* UPDATE LASER ******************************************** " << std::endl;
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  cout << "(Debug) - x_ = " << x_ << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  cout << " ******************************************* UPDATE RADAR ******************************************** " << std::endl;

  // Create the h matrix
  VectorXd z_pred(3);
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // z_pred << sqrt(pow(px,2) + pow(py,2)),
  //               atan(py / px),
  //               (px * vx + py * vy) / (sqrt(pow(px,2) + pow(py,2)));

  z_pred << sqrt(pow(px,2) + pow(py,2)),
                atan2(py , px),
                (px * vx + py * vy) / (sqrt(pow(px,2) + pow(py,2)));

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  // Hazem Debug
  cout << "(Debug) - z_pred= " << z_pred << std::endl;
  cout << "(Debug) - z= " << z << std::endl;
  cout << "(Debug) - z_pred_angle_deg = "  << atan2(py , px) * 180 / 3.14 << std::endl;
  cout << "(Debug) - z_GT_angle_deg = "  << z[1] * 180 / 3.14 << std::endl;
  cout << "(Debug) - y= " << y << std::endl;
  cout << "(Debug) - K= " << K << std::endl;
  cout << "(Debug) - K*y= " << K*y << std::endl;
  cout << "(Debug) - x_ " << x_ << std::endl;
  

}
