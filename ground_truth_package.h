#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"
class GroundTruthPackage{
public:
  long long timestamp;
  enum SensorType{Radar,Lader} sensor_type_;
  Eigen::VectorXd gt_values_;
};
#endif