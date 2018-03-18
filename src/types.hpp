#pragma once

#include <vector>

using namespace std;

struct Point {
  double x;
  double y;
};

struct Car {
  int id;
  double x, y, s, d, yaw, speed;
};

struct Path {
  vector<double> x, y;
  double end_s, end_d;
};

struct SensorFusion {
  vector<int> id;
  vector<double> x;
  vector<double> y;
  vector<double> vx;
  vector<double> vy;
  vector<double> s;
  vector<double> d;
};
