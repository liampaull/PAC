#ifndef COVERAGE_SENSOR_H
#define COVERAGE_SENSOR_H

#include <vector>

class CoverageSensor{
 public:
  CoverageSensor() {};
  ~CoverageSensor() {};
  void initialize(std::vector<double> _f, double _range, double _resolution){
    f = _f;
    range = _range;
    resolution = _resolution;
  };

 public:
  std::vector<double> f;
  double         resolution;
  double         range;
};

#endif
