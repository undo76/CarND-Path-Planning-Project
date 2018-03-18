#pragma once

#include <vector>

using namespace std;
const double MPH = 0.44704;
const double DELTA_T = 0.02;
const double MAX_SPEED = 49.5 * MPH; // Allow some margin.
const double MAX_ACCEL = 10 * 0.5;  // m/s^2
const double MAX_JERK = 10 * 0.9;   // m/s^3

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
