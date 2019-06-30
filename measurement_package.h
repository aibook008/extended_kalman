#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_
#include "Eigen/Dense"
class measurement_package{
public:
  long long timestamp;
  enum SensorType{Radar,Lader} sensor_type_;
  Eigen::VectorXd rawmeasurements;
};


#endif
// data type for sensor measurement usually includes timsstamp, sensor type, data(expressed in vector/eigen library)