#include "Cars.hpp"
#include "utils.hpp"
#include <iostream>

Cars::Cars() {
  for (int i = 0; i < N_LANES; i++) {
    vector<Car> cars_in_lane;
    cars.push_back({});
  }
}

void Cars::update(const SensorFusion sf) {
  for (int i = 0; i < N_LANES; i++) {
    cars[i] = {};
  }
  for (int i = 0; i < sf.id.size(); i++) {
    Car car = {.id = sf.id[i],
               .x = sf.x[i],
               .y = sf.y[i],
               .s = sf.s[i],
               .d = sf.d[i],
               .speed = hypo(sf.vx[i], sf.vy[i])};
    for (int l = 0; l < N_LANES; l++) {
      if (isCarInLane(car, l)) {
        cars[l].push_back(car);
      }
    }
  }
}

bool Cars::isCarInLane(const Car &car, int lane_n) {
  // I leave a margin of 1m. Therefore a car can be in more than a lane at
  // a time.
  return lane(lane_n) - 3 < car.d && car.d < lane(lane_n) + 3;
}

Car Cars::nextCarInLane(const double s, int lane_n) {
  Car next_car = {.id = -1, .s = 999999, .speed = 100};
  double min_distance = 1000;
  for (Car c : cars[lane_n]) {
    double distance = mod(c.s - s, CIRCUIT_LENGTH);
    if (min_distance > distance) {
      next_car = c;
      min_distance = distance;
    }
  }
  return next_car;
}

Car Cars::previousCarInLane(const double s, int lane_n) {
  Car next_car = {.id = -1, .s = 999999, .speed = 100};
  double min_distance = 1000;
  for (Car c : cars[lane_n]) {
    double distance = mod(s - c.s, CIRCUIT_LENGTH);
    if (min_distance > distance) {
      next_car = c;
      min_distance = distance;
    }
  }
  return next_car;
}

bool Cars::isLaneSafe(const Car car, int lane_n, double margin) {
  Car next_car = nextCarInLane(car.s, lane_n);
  Car previous_car = previousCarInLane(car.s, lane_n);

  const double t = 1.5; // seconds

  return (next_car.id == -1 ||
          (future_s(next_car, .01) - future_s(car, .01) > margin)) &&
         (previous_car.id == -1 ||
          (future_s(car, .01) - future_s(previous_car, .01) > margin)) &&
         (next_car.id == -1 ||
          (future_s(next_car, t) - future_s(car, t) > margin)) &&
         (previous_car.id == -1 ||
          (future_s(car, t) - future_s(previous_car, t) > margin));
}

double Cars::future_s(const Car car, const double time) {
  return car.s + car.speed * time;
}