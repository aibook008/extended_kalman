#ifndef FusionEKF_H_
#define FusionEKF_H_
#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"
class FusionEKF{
public:
  //initiate all the member variable
  FusionEKF();
  ~FusionEKF();
  void ProcessMeasuremnt(const measurement_package &meas);
  KalmanFilter ekf_;
private:
  bool is_initiated_;
  long previous_timestamp;
  //measurement matrix radar, lidar && H laser, Hj_For radar
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser;
  Eigen::MatrixXd Hj_;
  
  
};

#endif