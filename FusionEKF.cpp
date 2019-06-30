#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEKF::FusionEKF(){
 //initiate related parameters
  is_initiated_=false;
  previous_timestamp=0;
  R_laser_=MatrixXd(2,2);
  R_radar_=MatrixXd(3,3);  
  H_laser=MatrixXd(2,4);
  Hj_=MatrixXd(3,4);
  
  R_laser_<<0.0225, 0, 0, 0.0225;
  R_radar_<<0.09, 0, 0,0, 0.0009, 0,0, 0, 0.09;
  H_laser << 1, 0, 0, 0,0, 1, 0, 0;
  Hj_ << 1, 1, 0, 0,1, 1, 0, 0,1, 1, 1, 1;
  //initiation of kalman member element
  ekf_.P_=MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0, 0, 1, 0, 0,0, 0, 1000, 0,0, 0, 0, 1000;
  ekf_.F_=MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,0, 1, 0, 1,0, 0, 1, 0,0, 0, 0, 1; 

}

FusionEKF::~FusionEKF(){}
void FusionEKF::ProcessMeasuremnt(const measurement_package& meas)
{if(!is_initiated_){
 //initate the x with the first measurment data// initiate the covariance matrix// tansform the polar coordinateion 
  //to cartisen coordination
  ekf_.x_=VectorXd(4);
  ekf_.x_<<1,1,1,1;
  if (meas.sensor_type_==measurement_package::Lader)
  {ekf_.x_(0)=meas.rawmeasurements(0);
    ekf_.x_(1)=meas.rawmeasurements(1);   
  }
  else if(meas.sensor_type_==measurement_package::Radar){
  float ro = meas.rawmeasurements(0);
  float phi=meas.rawmeasurements(1);
  float ro_dot=meas.rawmeasurements(2);
  ekf_.x_(0)=ro*cos(phi);
  ekf_.x_(1)=ro*sin(phi);
  ekf_.x_(2)=ro_dot*cos(phi);
  ekf_.x_(3)=ro_dot*sin(phi);
  
  }
  previous_timestamp=meas.timestamp;
  is_initiated_=true;
  return;}  
  //prediction step
  float dt=(meas.timestamp-previous_timestamp)/1000000.0;
  previous_timestamp=meas.timestamp;
  float dt_2=dt*dt;
  float dt_3=dt_2*dt;
  float dt_4=dt_3*dt;
  //update the F matrix, state transition matrix
  ekf_.F_(0,2)=dt;
  ekf_.F_(1,3)=dt;
  float noise_ax=9;float noise_ay=9;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0, 0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0, 0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;            
  ekf_.predict();
//update 
  if (meas.sensor_type_==measurement_package::Lader)
  {//assign number to related member
    ekf_.H_=H_laser;
    ekf_.R_=R_laser_;
    ekf_.udpate(meas.rawmeasurements);
   
  }
  else if(meas.sensor_type_==measurement_package::Radar)
  {Tools Tool;
   Hj_=Tool.CalculateJacub(ekf_.x_);
   ekf_.H_=Hj_;
   ekf_.R_=R_radar_;
   ekf_.updateEKF(meas.rawmeasurements);  
    
  }
  
  
}




