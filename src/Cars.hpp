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
  Car nextCarInLane(const double s, int lane_n);
  Car previousCarInLane(const double s, int lane_n);
  bool isLaneSafe(const Car car, int lane_n, double margin);

private:
  vector<vector<Car>> cars;
  double future_s(const Car car, const double time);
};
