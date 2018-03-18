#include "Cars.hpp"
#include "utils.hpp"
#include <iomanip>
#include <iostream>
#include <math.h>

Cars::Cars() {
  for (int i = 0; i < N_LANES; i++) {
    vector<Car> cars_in_lane;
    lanes.push_back({});
  }
}

void Cars::update(const SensorFusion sf) {
  for (int i = 0; i < N_LANES; i++) {
    lanes[i] = {};
  }
  for (int i = 0; i < sf.id.size(); i++) {
    Car car = {.id = sf.id[i],
               .x = sf.x[i],
               .y = sf.y[i],
               .s = sf.s[i],
               .d = sf.d[i],
               .yaw = 0,
               .speed = hypo(sf.vx[i], sf.vy[i])};
    for (int l = 0; l < N_LANES; l++) {
      if (isCarInLane(car, l)) {
        lanes[l].push_back(car);
      }
    }
  }
}

bool Cars::isCarInLane(const Car &car, int lane_n) {
  // I leave a margin of 1m. Therefore a car can be in more than a lane at
  // a time.
  return lane_to_d(lane_n) - 3 < car.d && car.d < lane_to_d(lane_n) + 3;
}

Car Cars::nextCarInLane(const double s, int lane_n) {
  Car next_car = {
      .id = -1, .x = 0, .y = 0, .s = 999999, .d = 0., .yaw = 0., .speed = 100};
  double min_distance = 1000;
  for (Car c : lanes[lane_n]) {
    double distance = mod(c.s - s, CIRCUIT_LENGTH);
    if (min_distance > distance) {
      next_car = c;
      min_distance = distance;
    }
  }
  return next_car;
}

Car Cars::previousCarInLane(const double s, int lane_n) {
  Car next_car = {
      .id = -1, .x = 0, .y = 0, .s = 999999, .d = 0., .yaw = 0., .speed = 100};
  double min_distance = 1000;
  for (Car c : lanes[lane_n]) {
    double distance = mod(s - c.s, CIRCUIT_LENGTH);
    if (min_distance > distance) {
      next_car = c;
      min_distance = distance;
    }
  }
  return next_car;
}

bool Cars::isLaneSafe(const Car car, const int lane_n, const double margin,
                      const double time) {
  Car next_car = nextCarInLane(car.s, lane_n);
  Car previous_car = previousCarInLane(car.s, lane_n);

  return (next_car.id == -1 ||
          (future_s(next_car, time) - future_s(car, time) > margin)) &&
         (previous_car.id == -1 ||
          (future_s(car, time) - future_s(previous_car, time) > margin));
}

double Cars::future_s(const Car car, const double time) {
  return car.s + car.speed * time;
}

double Cars::cost(const Car car, int target_lane, int lane_n) {
  Car next_car = nextCarInLane(car.s, lane_n);
  Car previous_car = previousCarInLane(car.s, lane_n);

  double speed_cost = max(MAX_SPEED - next_car.speed, 0.);
  double distance_cost = 100 * 1 / (next_car.s - car.s);
  double change_cost = max((double)abs(target_lane - lane_n), 1.5);

  double total = speed_cost + distance_cost + change_cost;

  cout << fixed << setprecision(2);
  cout << lane_n << ":\t" << speed_cost << "\t" << distance_cost << "\t"
       << change_cost << "\t"
       << " = " << total << "\t" << endl;
  return total;
}