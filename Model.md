# Model documentation

## Speed control

The speed of the car is controlled with a PID. I use the acceleration as a the controlled variable. To prevent excesive acceleration this value is trimmed to the maximum acceleration.

``` c++
void PathPlanifier::regulateSpeed() {
  acceleration = -max(
      -MAX_ACCEL, min(MAX_ACCEL, speed_control.update(target_speed - speed)));
  speed = speed + acceleration * DELTA_T;

}
```

The parameters of the PID are:

<table>
  <tr>
    <th>K_p</th>
    <th>K_i</th>
    <th>K_d</th>
  </tr>
  <tr>
    <td>.7</td>
    <td>.0000001</td>
    <td>.1</td>
  </tr>
</table>

## Path plannifier

If the car before the ego car is far away we keep the lane at maximum speed. Otherwise we need to select the most promising lane, i.e. the one that allows to advance faster.

The path planifier consist of three phases:

  1. Lane cost calculation.
  2. Selection of lane.
  3. Target speed calculation.

### Lane cost

In order to select the best lane an heuristic is calculate in order to select the most promising lane of the road according to the next criteria: 

  - **Speed**: Speed of the next car in the lane
  - **Distance**: Inverse of the distance to the next car
  - **Change of lanes**: The number of lanes between the current lane and the selected one.

  These costs are scaled and added to calculate a total cost for the lane.

  ``` cpp
  
  double speed_cost = max(MAX_SPEED - next_car.speed, 0.);
  double distance_cost = 300 * 1 / (next_car.s - car.s);
  double change_cost = abs(target_lane - lane_n) * 3;

  double total = speed_cost + distance_cost + change_cost;

  ```

### Selection of lane

The cost of each line is not enough to select the line. We need also to know if we can arrive to the selected lane safely. In order to do we project in the near future the positions of the previous car and the next car for each lane.

We only select a lane if the distance with other cars is rescpected for different times in the future.

### Selection of speed

In order to select the final speed, we only take into account the speed and the distance of the next car in the selected lane. If the car is far away, we select maximum speed. Otherwise, we will reduce speed with regard to the next car.

