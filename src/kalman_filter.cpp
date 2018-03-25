#include "kalman_filter.h"

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

// cartesian to polar conversion
VectorXd cartesian2Polar(VectorXd cartesian){
  double rho, phi, rho_n;
  double px, py, vx, vy;
  Eigen::VectorXd polar(3);
  
  px = cartesian(0);
  py = cartesian(1);
  vx = cartesian(2);
  vy = cartesian(3);
  
  rho = sqrt(px*px + py*py);
  rho_n = (px*vx + py*vy)/rho;
  phi = atan2(py, px);
  
  polar << rho, phi, rho_n;
  return polar;
}

// convert polar coordinates to cartesian
VectorXd polar2Cartesian(VectorXd polar){
  double px, py, vx, vy;
  double phi, rho, rho_n;
  VectorXd cartesian(4);
  
  rho = polar(0);
  phi = polar(1);
  rho_n = polar(2);
  
  px = rho * cos(phi);
  py = rho * sin(phi);
  
  vx = rho_n * cos(phi);
  vy = rho_n * sin(phi);
  
  cartesian << px, py, vx, vy;
  
  return cartesian;
}

// normalise the angle to be in range -pi to pi
double normAngle(double theta){
    double f = ((int)(theta/6.28))*6.28;
    if(f == 0)
        return theta;
    if(theta < 0)
        return theta + f;
    return theta - f;
}



void KalmanFilter::Predict() {
  /*
   * predict the state
   */
  x_ = F_ * x_;
  P_ = (F_ * P_ * F_.transpose()) + Q_;
}

void KalmanFilter::UpdateState(const VectorXd y){
  /*
   * common update for lidar and radar
   */
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  // updated estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}


void KalmanFilter::Update(const VectorXd &z) {
  /*
   * update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_* x_;
  UpdateState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*
   * update the state by using Extended Kalman Filter equations
   */
  VectorXd Hx = cartesian2Polar(x_);
  
  VectorXd y = z - Hx;
  y(1) = normAngle(y(1));
  
  UpdateState(y);
}
