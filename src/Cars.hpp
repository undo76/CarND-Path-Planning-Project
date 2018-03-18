#pragma once

#include "types.hpp"
#include <vector>

using namespace std;

const int N_LANES = 3;
const double CIRCUIT_LENGTH = 6945.554;

class Cars {
public:
  Cars();

  void update(const SensorFusion sensor_fusion);
  bool isCarInLane(const Car &car, int lane_n);
  Car nextCarInLane(double s, int lane_n);
  Car previousCarInLane( double s, int lane_n);
  bool isLaneSafe(const Car car, int lane_n, double margin, double time);
  double cost(const Car car, int target_lane, int lane_n);
  

private:
  vector<vector<Car>> lanes;
  double future_s(const Car car, const double time);
};
