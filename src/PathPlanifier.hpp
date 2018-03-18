#pragma once

#include "Cars.hpp"
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "types.hpp"
#include "PID.h"
#include <iostream>

using namespace std;

const double MPH = 0.44704;
const double DELTA_T = 0.02;
const double MAX_SPEED = 49.5 * MPH; // Allow some margin.
const double MAX_ACCEL = 10 * 0.5;  // m/s^2
const double MAX_JERK = 10 * 0.9;   // m/s^3

// Spline control points
const int SPL_CTL_DIST = 40; // Distance between adyacent control points
const int SPL_LENGTH = 120;  // Distance to the last control point

class PathPlanifier {
public:
  PathPlanifier(const vector<double> &map_waypoints_x,
                const vector<double> &map_waypoints_y,
                const vector<double> &map_waypoints_s,
                const vector<double> &map_waypoints_dx,
                const vector<double> &map_waypoints_dy)
      : map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y),
        map_waypoints_s(map_waypoints_s), map_waypoints_dx(map_waypoints_dx),
        map_waypoints_dy(map_waypoints_dy){};

  vector<vector<double>> update(const Car &car, const Path &previous_path,
                                const SensorFusion &sensor_fusion);

private:
  const vector<double> &map_waypoints_x;
  const vector<double> &map_waypoints_y;
  const vector<double> &map_waypoints_s;
  const vector<double> &map_waypoints_dx;
  const vector<double> &map_waypoints_dy;

  Car car;
  Path previous_path;
  Cars cars;
  PID speed_control = PID(.7, .0000001, .1);

  double target_speed = MAX_SPEED;
  double speed = 0.; // Speed at the end of the path.
  double acceleration = 0.;
  int target_lane = 1;

  Point toXY(const double s, const double d);
  Point toCarCoordinates(const Point &point, const Car &car);
  Point fromCarCoordinates(const Point &point, const Car &car);
  tk::spline toSpline(const double lane, const Car &car);
  void regulateSpeed();
  vector<vector<double>> calculatePath();
  int findNextCarInLane(const double s, const double lane);
  void selectLaneAndSpeed();
};
