#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
  
  std::cout << "before Predict" << std::endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ *P_ *Ft+Q_;
  
  std::cout << "after Predict" << std::endl;

  // in vidoes x has an extra u
  /*
  x = F * x + u;
  MatrixXd Ft = F.transpose();
  P = F * P * Ft + Q;
  */

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // In section 7 of lesson 5
  /*
  VectorXd y = z - H * x;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P * Ht + R;
  Matrix Si = S.inverse();
  MatrixXd K = P * Ht * Si;
  */

  std::cout << "before Update" << std::endl;


  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  std::cout << "after update" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Section 14 of lesson 5

  std::cout << "before updateekf" << std::endl;
  
  KalmanFilter ekf_;

  float x = ekf_.x_(0);
  float y = ekf_.x_(1);
  float vx = ekf_.x_(2);
  float vy = ekf_.x_(3);

  float rho = sqrt(x*x+y*y);
  float theta = atan2(y,x);
  float ro_dot = (x*vy+y*vy)/rho;
  VectorXd x_pred = VectorXd(3);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;

  
 
  // In section 7 of lesson 5
  
  MatrixXd H_; // measurement matrix;

  MatrixXd I;
  MatrixXd Q;
  MatrixXd F;
  MatrixXd P;
  MatrixXd R;
  VectorXd u;

  u = VectorXd(2);
  u << 0, 0;

  P = MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  
 
  VectorXd y_ = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P * Ht * Si;

  //new state
  //x = VectorXd(2);
  //x << 0, 0;

  x_ = x_ + (K * y);
  P_ = (I - K + H_) * P_;

  //KF Prediction step

  

  x_ = F * x_ + u;
  MatrixXd Ft = F.transpose();
  P_ = F_ * P_ * Ft +Q_;
  
  std::cout << "before updateekf" << std::endl;

}
