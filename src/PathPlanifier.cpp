#include "PathPlanifier.hpp"
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "utils.hpp"
#include <iostream>
#include <math.h>

const int N_POINTS = 50;

vector<vector<double>>
PathPlanifier::update(const Car &car, const Path &previous_path,
                      const SensorFusion &sensor_fusion) {
  auto pp = this->previous_path;
  this->car = car;
  this->previous_path = previous_path;

  cars.update(sensor_fusion);

  selectLaneAndSpeed();

  return calculatePath();
}

void PathPlanifier::selectLaneAndSpeed() {
  double s = car.s;

  Car selected = cars.nextCarInLane(s, target_lane);

  // If the car in front is far, continue at full speed
  if (mod(selected.s - s, CIRCUIT_LENGTH) > 60) {
    target_speed = MAX_SPEED;
  } else {
    int selected_lane = target_lane;
    double selected_speed = MAX_SPEED;
    for (int l = 0; l < N_LANES; l++) {
      Car next = cars.nextCarInLane(s, l);
      if (l == target_lane) {
        if (mod(next.s - s, CIRCUIT_LENGTH) < 30) {
          selected_speed = min(selected.speed, MAX_SPEED);
        } else if (mod(next.s - s, CIRCUIT_LENGTH) < 10) {
          selected_speed = min(selected.speed - 2 * MPH, MAX_SPEED);
        }
        continue;
      }

      if (cars.isLaneSafe(car, l, 7)) {
        if (abs(target_lane - l) <= 1 ||
            cars.isLaneSafe(car, target_lane > l ? l + 1 : l - 1, 7)) {

          // A free lane: Go there.
          if (mod(next.s - s, CIRCUIT_LENGTH) > 200) {
            selected_lane = l;
            selected = next;
            selected_speed = MAX_SPEED;
            break;

            // A lane with a far car, take it.
          } else if (mod(next.s - s, CIRCUIT_LENGTH) > 60) {
            selected_lane = l;
            selected = next;
            selected_speed = MAX_SPEED;

            // A lane with a car at medium distance, select as provisional best
          } else if (mod(next.s - s, CIRCUIT_LENGTH) > 20 && next.s > selected.s) {
            selected_lane = l;
            selected = next;
            selected_speed = MAX_SPEED;

            // A lane with a close car moving faster that the current one.
          } else if (mod(next.s - s, CIRCUIT_LENGTH) > 10 &&
                     next.speed > selected.speed) {
            selected_lane = l;
            selected = next;
            selected_speed = min(next.speed, MAX_SPEED);
          }
        }
      }
    }
    target_speed = selected_speed;
    target_lane = selected_lane;
  }
}

vector<vector<double>> PathPlanifier::calculatePath() {
  vector<double> next_x_vals, next_y_vals;

  const double path_size = previous_path.x.size();

  // Copy previous path
  next_x_vals.insert(next_x_vals.begin(), previous_path.x.begin(),
                     previous_path.x.end());
  next_y_vals.insert(next_y_vals.begin(), previous_path.y.begin(),
                     previous_path.y.end());

  // Calculate spline from last point of previous path.
  Car ref;
  if (path_size >= 2) {
    ref.x = previous_path.x[path_size - 1];
    ref.y = previous_path.y[path_size - 1];
    double delta_x = (ref.x - previous_path.x[path_size - 2]);
    double delta_y = (ref.y - previous_path.y[path_size - 2]);
    ref.yaw = atan2(delta_y, delta_x);
    ref.s = previous_path.end_s;
    ref.d = previous_path.end_d;
  } else {
    ref = car;
  }

  auto spline = toSpline(target_lane, ref);

  // Add new points
  Point next;
  double x = 0, y = 0;
  for (int i = 0; i < N_POINTS - path_size; i++) {
    regulateSpeed();

    double x_p = x, y_p = y;
    x = x_p + 1;
    y = spline(x);

    // Correction to take in account the lateral displacement.
    double h = hypo(1, y - y_p);
    x = x_p + (1. / h) * speed * DELTA_T;
    y = spline(x);
    next = fromCarCoordinates({x, y}, ref);
    next_x_vals.push_back(next.x);
    next_y_vals.push_back(next.y);
  }

  return {next_x_vals, next_y_vals};
}

Point PathPlanifier::toXY(const double s, const double d) {
  return getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
}

Point PathPlanifier::toCarCoordinates(const Point &p, const Car &ref) {
  double x_shift = p.x - ref.x;
  double y_shift = p.y - ref.y;

  double x_rot = x_shift * cos(-ref.yaw) - y_shift * sin(-ref.yaw);
  double y_rot = x_shift * sin(-ref.yaw) + y_shift * cos(-ref.yaw);

  return {x_rot, y_rot};
}

Point PathPlanifier::fromCarCoordinates(const Point &p, const Car &ref) {
  double x_rot = p.x * cos(ref.yaw) - p.y * sin(ref.yaw);
  double y_rot = p.x * sin(ref.yaw) + p.y * cos(ref.yaw);

  double x_shift = x_rot + ref.x;
  double y_shift = y_rot + ref.y;

  return {x_shift, y_shift};
}

tk::spline PathPlanifier::toSpline(const double target_lane, const Car &ref) {
  // Spline control points
  vector<double> pts_x;
  vector<double> pts_y;

  // Add control points to make spline tangent to current trajectory.
  // Create a control point one meter before along ref's direction.

  pts_x.push_back(-.5);
  pts_y.push_back(0.);
  pts_x.push_back(0.);
  pts_y.push_back(0.);

  // Add control points
  for (int i = SPL_CTL_DIST; i <= SPL_LENGTH; i += SPL_CTL_DIST) {
    Point p = toXY(ref.s + i, lane(target_lane));
    Point p_car = toCarCoordinates(p, ref);
    pts_x.push_back(p_car.x);
    pts_y.push_back(p_car.y);
  }

  // Generate spline
  tk::spline spline;
  spline.set_points(pts_x, pts_y);
  return spline;
}

void PathPlanifier::regulateSpeed() {
  acceleration = -max(
      -MAX_ACCEL, min(MAX_ACCEL, speed_control.update(target_speed - speed)));
  speed = speed + acceleration * DELTA_T;

  /*
  if (speed < target_speed) {
    // accelerate
    acceleration = min(acceleration + MAX_JERK * DELTA_T, MAX_ACCEL);
    // acceleration = min(acceleration, (target_speed - speed) * acceleration);
    speed = min(speed + acceleration * DELTA_T, target_speed);
  } else {
    // decelerate
    acceleration = max(acceleration - MAX_JERK * DELTA_T, -MAX_ACCEL);
    // acceleration = max(acceleration, (target_speed - speed) * acceleration);
    speed = max(speed + acceleration * DELTA_T, target_speed);
  }*/
}