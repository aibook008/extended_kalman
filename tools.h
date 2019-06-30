#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
using namespace std;
class Tools{
public:
  Tools();
  virtual ~Tools();
  Eigen::MatrixXd CalculateJacub(const Eigen::VectorXd &x);
  Eigen::VectorXd CalculateRSME(const vector<Eigen::VectorXd> &esti,const vector<Eigen::VectorXd> &groudth);
};
#endif