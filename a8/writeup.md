# Trajectory Generation

> 1. how is lane following achieved? (1points)

Lane following is achieved by keeping `d` constant in frenet coordinates. `s` values indicate the distance along the road, and the `d` values indicate the perpendicular distance from the ego vehicle and the center median. Keeping `d` constant, the frenet coordinates are then converted into carteisian coordinates for the ego vehicle to use.

> 2.  how to use spline to generate a smooth trajectory? (2point)

Using basic interpolation methods to generate a trajectory between two waypoints leads to very angular trajectories. To create a smoother trajectory, a polynomial interpolation is used instead; a spline. A spline is created using sparse waypoints spaced apart by a constant distance. Between these sparse waypoints an additional `N` equally spaced waypoints are created (N determined by the velocity and the target distance). Each of the `N` additional waypoints points are created by sampling the spline using the equally distanced point. By sampling the spline, a smooth trajectory with `N` waypoints is created between the original two sparse waypoints.

> 3. how to avoid collision with the car in front? (0.5points)

Using the sensor data, the lane, distance from the ego vehicle and velocity of other cars are known. Using this information, the future position of the vehicle is calculated and if it is above a threshold the lane is classified as occupied. If the lane of the ego vehicle is occupied, it will switch lanes if they are also not occupied, or else it will slow down by reducing its speed by a constant value (small value to minimize jerk).

> 4.  how to avoid cold start? (0.5points)

A cold start is avoided by initialing the velocity to `0 mph`. On every path update, the velocity is increased slightly until a maximum of `49.5 mph`.

# Behavior Planning

> Briefly explainv your approach for behavior planning and any modifications to the provided trajectory generation code (7-10sentences)

To achieve lane-switching, a modification to the `too_close` logic was made.
Instead of checking just the current lane of the ego vehicle, all lanes are checked to see if they are free.
Here `free` refers to if the `s` position of another vehicle in that lane is not within 30 meters in front of the ego vehicle,
or if lane is not the same as the ego vehicle (lane switching) then another check of 15 meters behind the ego vehicle.
If the lane of the ego vehicle is occupied, it will check if it can switch lanes and if those lanes are also not occupied.
If it can switch lanes, the new lane is supplied as input to the spline trajectory.
If it cannot switch lanes, the speed of the ego vehicle is reduced to avoid collision.

```c++
bool isLaneFree(
    vector<vector<double>> sensor_fusion,
    double car_s,
    int car_lane,
    int new_lane,
    int prev_size)
{
  if (new_lane > MAX_LANE || new_lane < MIN_LANE)
    return false;
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // Check if the car is in the same lane as the new lane
    float d = sensor_fusion[i][6];
    if (d < (LANE_WIDTH / 2 + LANE_WIDTH * new_lane + LANE_WIDTH / 2) && d > (LANE_WIDTH / 2 + LANE_WIDTH * new_lane - LANE_WIDTH / 2))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];
      // Calculate the check_car's future location
      check_car_s += (double)prev_size * INTERVAL * check_speed;

      // If the check_car is within 30 meters in front, reduce ref_vel so that we don't hit it
      if (check_car_s > car_s && (check_car_s - car_s) < WAYPOINT_SPACING / 2)
        return false;
      // If we are switching lanes, we need to check out blind spot ;)
      if (car_lane != new_lane && car_s > check_car_s && (car_s - check_car_s) < WAYPOINT_SPACING / 6)
        return false;
    }
  }
  return true;
}
int main()
{
    ...

    bool isForwardFree = isLaneFree(sensor_fusion, car_s, lane, lane, previous_path_x.size());
    bool isLeftFree = isLaneFree(sensor_fusion, car_s, lane, lane - 1, previous_path_x.size());
    bool isRightFree = isLaneFree(sensor_fusion, car_s, lane, lane + 1, previous_path_x.size());
    bool shouldSlowDown = false;
    if (!isForwardFree)
    {
        if (isLeftFree)
        {
            lane -= 1;
        }
        else if (isRightFree)
        {
            lane += 1;
        }
        else
        {
            shouldSlowDown = true;
        }
    }

    ...

}
```
