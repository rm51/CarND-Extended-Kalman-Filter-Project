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
  cout << "x_" << endl << x_ << endl << endl;
  cout << "F_" << endl << x_ << endl << endl;

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();

  cout << "P_" << endl << x_ << endl << endl;
 cout << "Q_" << endl << x_ << endl << endl;
    P_ = F_ *P_ *Ft+Q_;
  
  std::cout << "after Predict" << std::endl;

  
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

  // commenting out do we need this? 
  /*
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  */
  MatrixXd K = P_ * Ht * Si;
  
  
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

  std::cout << "before update ekf" << std::endl;
  
  // KalmanFilter ekf_;

  std::cout << "KalmanFilter" << std::endl;
  // error with x 
  float x = ekf_.x_(0);

  std::cout << "after x" << std::endl;
  float y = ekf_.x_(1);
  std::cout << "after y" << std::endl;
  float vx = ekf_.x_(2);
  std::cout << "after vx" << std::endl;
  float vy = ekf_.x_(3);
  std::cout << "after vy" << std::endl;

  std::cout << "before rho" << std::endl;

  float rho = sqrt(x*x+y*y);
  float theta = atan2(y,x);
  float ro_dot = (x*vy+y*vy)/rho;
  VectorXd x_pred = VectorXd(3);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;

  
 
  // In section 7 of lesson 5
  MatrixXd P;
  
 std::cout << "before define matrix i" << std::endl;

 // define Matrix I to be the Identity matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

 

  P = MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  
 
  VectorXd y_ = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new state
  //x = VectorXd(2);
  //x << 0, 0;

  std::cout << "before  x_= x_ + (K * y_)" << std::endl;

  x_ = x_ + (K * y_);
  P_ = (I - K * H_) * P_;

  //KF Prediction step

  
  /*
  x_ = F * x_ + u;
  MatrixXd Ft = F.transpose();
  P_ = F_ * P_ * Ft +Q_;
  */
  
  std::cout << "after updateekf" << std::endl;

}
