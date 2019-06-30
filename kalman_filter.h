#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
using Eigen::VectorXd;
using Eigen::MatrixXd;
//kalman filter includes kalman vector intiaition/state prediction/state update/EKF matrix update
class KalmanFilter{
  public:
  VectorXd x_ ;
  MatrixXd P_;
  MatrixXd Q_;
  MatrixXd R_; 
  MatrixXd F_; 
  MatrixXd H_;
  KalmanFilter();
 virtual ~KalmanFilter();
 //initiate all related vector & matrix
 void init(VectorXd &x,MatrixXd &P,MatrixXd &Q,MatrixXd &R,MatrixXd &F,MatrixXd &H);
 void predict();
 void udpate(const VectorXd &z);//update kalman filter
 void updateEKF(const VectorXd &z);//update extended filter
 
  

  
};
#endif 