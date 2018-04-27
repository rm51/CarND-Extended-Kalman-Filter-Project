#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
}

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

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
    P_ = F_ *P_ *Ft+Q_;

}

void KalmanFilter::Update(const VectorXd &z) {


  // In section 7 of lesson 5

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();

  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}



void KalmanFilter::UpdateEKF(const VectorXd &z) {

    float x = x_(0);
    Tools tools;
    MatrixXd Hj = tools.CalculateJacobian(x_);

    float y = x_(1);

  float vx = x_(2);

  float vy = x_(3);


  float rho = sqrt(x*x+y*y);
  float phi = atan2(y,x);
  float ro_dot =  (x*vx + y*vy) / rho;
  VectorXd x_pred(3);

  VectorXd z_pred(3);
  z_pred << rho, phi, ro_dot;

 // define Matrix I to be the Identity matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  VectorXd y_(3);
  // Normalize angle after y_
  y_ = z - z_pred;


    y_[1] = atan2(sin(y_[1]),cos(y_[1]));

    // Lesson 5 Part 20

    MatrixXd Ht = Hj.transpose();
    MatrixXd S = Hj * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = P_ * Ht * Si;


  x_ = x_ + (K * y_);
  P_ = (I - K * Hj) * P_;

}
