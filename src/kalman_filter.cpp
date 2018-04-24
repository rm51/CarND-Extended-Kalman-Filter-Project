#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
    u = VectorXd(2);
    u << 9, 9;
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
  /**
  TODO:
    * predict the state
  */
  
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

  VectorXd y(2);
  MatrixXd S(2,2);
  MatrixXd K(4,2);

  VectorXd z_pred = H_ * x_;
  y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();

  // commenting out do we need this? 

  K = P_ * Ht * Si;
  
  
  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_ = (I - K * H_) * P_;


  std::cout << "after update" << std::endl;
}



void KalmanFilter::UpdateEKF(const VectorXd &z) {

    std::cout << "before update ekf" << std::endl;

    float x = x_(0);

    std::cout << "after x" << std::endl;
    float y = x_(1);

    // added check for if px, py - negative
    if ((x < 0) && (y < 0))
    {
        x = fabs(x);
        y = fabs(y);
    }
    else if  (x < 0)
      x = fabs(x);
  std::cout << "after y" << std::endl;
  float vx = x_(2);
  std::cout << "after vx" << std::endl;
  float vy = x_(3);
  std::cout << "after vy" << std::endl;

  std::cout << "before rho" << std::endl;

  float rho = sqrt(x*x+y*y);
  float phi = atan2(y,x); // theta is phi
  float ro_dot = 0;
  VectorXd x_pred(3);


  if (fabs(rho)>=0.0001){
      ro_dot = (x*vy+y*vy)/rho;
  }
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


    std::cout << "after normalize angle" << std::endl;

    MatrixXd S(3,3);
    MatrixXd K(4,3);
    MatrixXd Ht = H_.transpose();

  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;

    S = H_ * P_ * Ht + R_;
    K = P_ * Ht * Si;

    std::cout << "before  x_= x_ + (K * y_)" << std::endl;

    x_ = x_ + (K * y_);
    P_ = (I - K * H_) * P_;
  
  std::cout << "after updateekf" << std::endl;



}
