#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include "FusionEKF.h"
#include "Eigen/Dense"
#include "measurement_package.h"
#include "ground_truth_package.h"
using namespace std;
using  Eigen::VectorXd;
using Eigen::MatrixXd;
void check_arg_count(int argc,char *argv[]){
  bool argc_valid=false;
  if (argc==3)
    argc_valid=true;
  else
    cout<<"input right parameters";
  if (!argc_valid)
  {exit(EXIT_FAILURE);}
  
}
void check_file_status(ifstream &in_file,string file,ofstream &out_file,string file2 ){
  if (!in_file.is_open()){
    cout<<"something wrong"<<file<<endl;
    exit(EXIT_FAILURE);
  }
  if(!out_file.is_open()){
    cerr<<"something wrong"<<file2<<endl;
    exit(EXIT_FAILURE);
  }
  
  
}
int main(int argc, char **argv) {
  //check_arg_count(argc,argv);
  //string infile_name=argv[1];
  cout << "Hello, world!" << endl;
  string infile_name="sample-laser-radar-measurement-data-1.txt";
  ifstream infile(infile_name.c_str(),ifstream::in);
  //string outfile_name=argv[2];
  ofstream outfile("output.txt",ofstream::out);
  if (!infile.is_open()||!outfile.is_open()){
    cout<<"file can not be open correctly"<<endl;
    exit(EXIT_FAILURE);
  }
  //check_file_status(infile,infile_name,outfile,outfile_name);
  vector<measurement_package> measurement_package_list;// measurement value read to measurment package list
  vector<GroundTruthPackage>  gt_pack_list;
  string line;
  Tools tool;
  while (getline(infile,line)){
    istringstream iss(line);
    measurement_package meas_package;
    GroundTruthPackage gt_package;
    string sensortype;
    iss>>sensortype;
    long long timestamp;
    if (sensortype.compare("R")==0){
      meas_package.sensor_type_=measurement_package::Radar;
      //initialize the vector dimension==3 for radar data
      meas_package.rawmeasurements=VectorXd(3);
      //radar has radical distance radical velocity and bearing angle(phi)
      float ro,phi,ro_dot;
      iss>>ro;iss>>phi;iss>>ro_dot;iss>>timestamp;
      meas_package.rawmeasurements<<ro,phi,ro_dot;
      meas_package.timestamp=timestamp;
      measurement_package_list.push_back(meas_package);
      
    }
    else if (sensortype.compare("L")==0){
      //update for lidar data which only have relative x &y
      meas_package.sensor_type_=measurement_package::Lader;
      meas_package.rawmeasurements=VectorXd(2);
      float x,y;iss>>x;iss>>y;iss>>timestamp;
      meas_package.rawmeasurements<<x,y;
      meas_package.timestamp=timestamp;
      measurement_package_list.push_back(meas_package);     
    }
    
    gt_package.gt_values_=VectorXd(4);
    float x,y, v_x,v_y;
    iss>>x;iss>>y;iss>>v_x;iss>>v_y;
    gt_package.gt_values_<<x,y,v_x,v_y;
    gt_pack_list.push_back(gt_package);
   // cout<<line<<endl;
    
  }
  FusionEKF fusionEKF;
  vector<VectorXd> estimations;
  vector<VectorXd> groundth;
  vector<measurement_package>::const_iterator ite;
  int k=0;//get groundtruth value
  for (ite=measurement_package_list.begin();ite!=measurement_package_list.end();ite++)
  {measurement_package meas_current=*ite;
  fusionEKF.ProcessMeasuremnt(meas_current);
  //cout<<fusionEKF.ekf_.x_(0)<<" :"<<std::endl;
  outfile<<fusionEKF.ekf_.x_(0)<<"\t";
 // cout<<fusionEKF.ekf_.x_(1)<<" :"<<std::endl;
  outfile<<fusionEKF.ekf_.x_(1)<<"\t";
 // cout<<fusionEKF.ekf_.x_(2)<<" :"<<std::endl;
  outfile<<fusionEKF.ekf_.x_(2)<<"\t";
  //cout<<fusionEKF.ekf_.x_(3)<<" :"<<std::endl;  
  outfile<<fusionEKF.ekf_.x_(3)<<"\t";
  //store the measurment value 
  if (meas_current.sensor_type_==measurement_package::Lader)
  {outfile<<meas_current.rawmeasurements[0]<<"\t";
    outfile<<meas_current.rawmeasurements[1]<<"\t";
  }
  else if (meas_current.sensor_type_==measurement_package::Radar)
  {float ro=meas_current.rawmeasurements[0];
    float phi=meas_current.rawmeasurements[1];
    outfile<<ro*cos(phi)<<"\t";
    outfile<<ro*sin(phi)<<"\t";
   }
   outfile<<gt_pack_list[k].gt_values_(0)<<"\t";
    outfile<<gt_pack_list[k].gt_values_(1)<<"\t";
     outfile<<gt_pack_list[k].gt_values_(2)<<"\t";
      outfile<<gt_pack_list[k].gt_values_(3)<<"\n";
    groundth.push_back(gt_pack_list[k].gt_values_);
      k=k+1;
   //push back value esitmation & groundtruth for further analysis
      estimations.push_back(fusionEKF.ekf_.x_);
     
      //VectorXd rsme;
  
  }
        VectorXd rsme=tool.CalculateRSME(estimations,groundth);
      cout<<"rsme:"<<rsme<<endl;
  cout<<"what happens"<<endl;
  return 0;
}
