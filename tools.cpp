#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools(){}
Tools::~Tools(){}
MatrixXd Tools::CalculateJacub(const VectorXd& x)
{MatrixXd Hj(3,4);
 float px=x(0);
 float py=x(1);
 float vx=x(2);
 float vy=x(3);
 float c1=px*px+py*py;
 float c2=sqrt(c1);
 float c3=c1*c2;
   if (fabs(c1) < 0.0001){
      std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
      return Hj;
  }
   Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;
 return Hj; 

}
VectorXd Tools::CalculateRSME(const vector< Eigen::VectorXd >& esti, const vector< Eigen::VectorXd >& groudth)
{Eigen::VectorXd rms(4);
  rms<<0,0,0,0;
  if (esti.size()!=groudth.size()||esti.size()==0)
  {std::cout<<"rsme something wrong, dimensions not equall"<<std::endl;
    return rms;
  }
  else{
    for (unsigned int i=0;i<esti.size();++i)
    {//std::cout<<esti[i]<<endl;
    //std::cout<<groudth[i]<<endl;
    Eigen::VectorXd residual(4);
    if (esti[i].size()==groudth[i].size()){
      residual=esti[i]-groudth[i];
      residual=residual.array()*residual.array();
    }
      else
      { residual<<0,0,0,0;}
     // 
      rms+=residual;
      //std::cout<<rms<<endl;
    }
    rms=rms/groudth.size();
    rms=rms.array().sqrt();
    return rms;
  }
  

}

