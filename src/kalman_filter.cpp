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
  
  std::cout << "before Predict" << std::endl;
  cout << "x_ Kalman Filter Predit" << endl << x_ << endl << endl;
  cout << "F_ Kalman Filter Predict" << endl << x_ << endl << endl;

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();

  cout << "P_ Kalman Filter Predict" << endl << x_ << endl << endl;
  cout << "Q_ Kalman Filter Predict" << endl << x_ << endl << endl;
    P_ = F_ *P_ *Ft+Q_;
  
  std::cout << "after Predict" << std::endl;

  
}

void KalmanFilter::Update(const VectorXd &z) {


  // In section 7 of lesson 5


  std::cout << "before Update" << std::endl;

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
   cout << "x_size" << endl;
   cout << x_size << endl;

  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


  std::cout << "after update" << std::endl;
}



void KalmanFilter::UpdateEKF(const VectorXd &z) {

    std::cout << "before update ekf" << std::endl;

    float x = x_(0);
    Tools tools;
    MatrixXd Hj = tools.CalculateJacobian(x_);

    std::cout << "after x" << std::endl;
    float y = x_(1);

  std::cout << "after y" << std::endl;
  float vx = x_(2);
  std::cout << "after vx" << std::endl;
  float vy = x_(3);
  std::cout << "after vy" << std::endl;

  std::cout << "before rho" << std::endl;


  // Check this part  -ro_dot, rewatch video


  float rho = sqrt(x*x+y*y);
  float phi = atan2(y,x); // theta is phi
  float ro_dot =  (x*vx + y*vy) / rho;
  VectorXd x_pred(3);

  /*
  if (rho>=0.0001){
      ro_dot = (x*vy+y*vy)/rho;
  }
   */
  VectorXd z_pred(3);
  z_pred << rho, phi, ro_dot;

  
 std::cout << "before define matrix i" << std::endl;

 // define Matrix I to be the Identity matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

    std::cout << "after define matrix i" << std::endl;

    std::cout << "before normalize angle" << std::endl;

  VectorXd y_(3);
  // TODO: Normalize angle after y_
  y_ = z - z_pred;
    std::cout << "before y_[1]" << endl;
    cout << y_[1] << std::endl;
    y_[1] = atan2(sin(y_[1]),cos(y_[1]));

    // Lesson 5 Part 20
    std::cout << "after normalize angle" << std::endl;

    MatrixXd Ht = Hj.transpose();
    MatrixXd S = Hj * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = P_ * Ht * Si;



  std::cout << "before  x_= x_ + (K * y_)" << std::endl;

  x_ = x_ + (K * y_);
  P_ = (I - K * Hj) * P_;
  
  std::cout << "after updateekf" << std::endl;



}
