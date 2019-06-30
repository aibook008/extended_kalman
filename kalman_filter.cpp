#include <math.h>
#include "kalman_filter.h"

KalmanFilter::KalmanFilter(){}
KalmanFilter::~KalmanFilter(){}
void KalmanFilter::predict(){
  x_=F_*x_;
  MatrixXd Ft=F_.transpose();
  P_=F_*P_*Ft+Q_;
  
}
void KalmanFilter::udpate(const VectorXd& z)
{VectorXd z_pred=H_*x_;
 VectorXd y=z-z_pred;
 MatrixXd Ht=H_.transpose();
 MatrixXd S=H_*P_*Ht+R_;
 MatrixXd k=P_*Ht*S.inverse();

 x_=x_+k*y;
 long x_size=x_.size();
 MatrixXd I=MatrixXd::Identity(x_size,x_size);
 P_=(I-k*H_)*P_;
//    VectorXd z_pred = H_ * x_;
//   VectorXd y = z - z_pred;
//   MatrixXd Ht = H_.transpose();
//   MatrixXd S = H_ * P_ * Ht + R_;
//   MatrixXd Si = S.inverse();
//   MatrixXd PHt = P_ * Ht;
//   MatrixXd K = PHt * Si;

}

void KalmanFilter::updateEKF(const VectorXd& z){
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  if (fabs(rho)<0.001)
  {rho_dot=0;}
  else
  {rho_dot=(x_(0)*x_(2) + x_(1)*x_(3))/rho;}
  VectorXd z_pred(3);
  z_pred<<rho,phi,rho_dot;
  
 VectorXd y=z-z_pred;
 MatrixXd Ht=H_.transpose();
 MatrixXd S=H_*P_*Ht+R_;
 MatrixXd k=P_*Ht*S.inverse();
 
 x_=x_+k*y;
 long x_size=x_.size();
 MatrixXd I = MatrixXd::Identity(x_size, x_size);
 P_ = (I - k * H_) * P_;
 

}

  



