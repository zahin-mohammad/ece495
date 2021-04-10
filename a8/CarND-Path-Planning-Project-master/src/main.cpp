#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const int WAYPOINT_SPACING = 30;
const int NUM_WAYPOINTS = 3;
const double LANE_WIDTH = 4;
const double INTERVAL = 0.02;
const int PLANNER_SIZE = 50;
const double MAX_DELTA_SPEED = 0.224;
const double SPEED_LIMIT = 49.5;
const double MS_TO_MPH = 2.24;
const int MAX_LANE = 2;
const int MIN_LANE = 0;

bool isLaneFree(
    vector<vector<double>> sensor_fusion,
    double car_s,
    int car_lane,
    int new_lane,
    int prev_size)
{
  if (new_lane > MAX_LANE || new_lane < MIN_LANE)
    return false;
  // Find ref_v to use
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
      check_car_s += (double)prev_size * 0.02 * check_speed;

      // If the check_car is within 30 meters in front, reduce ref_vel so that we don't hit it
      if (check_car_s > car_s && (check_car_s - car_s) < 30)
        return false;
      // If we are switching lanes, we need to check out blind spot ;)
      if (car_lane != new_lane && car_s > check_car_s && (car_s - check_car_s) < 30)
        return false;
    }
  }
  return true;
}

int main()
{
  uWS::Hub h;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  // Waypoints are spaced 30m apart
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Start in lane 1, which is the center lane (0 is left and 2 is right)
  int lane = 1;
  // Keeps track of if the car wants to move to a lane but can't
  int want_lane = -1;
  // Start at zero velocity and gradually accelerate
  double ref_vel = 0.0; // mph

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &lane, &want_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);
      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int prev_path_size = previous_path_x.size();
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = (prev_path_size <= 0) ? double(j[1]["s"]) : end_path_s;
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          /**
           * The given code defines a path made up of (x,y) points that the car
           *   will visit sequentially every .02 seconds
           */

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
          // Create a list of evenly spaced waypoints WAYPOINT_SPACING meters apart
          // Interpolate those waypoints later with spline and fill it in with more points
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x, y, yaw states, either will be the starting point or end point of the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if (prev_path_size < 2)
          {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - 0.5 * cos(car_yaw);
            double prev_car_y = car_y - 0.5 * sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use the previous path's end point as starting reference
          else
          {
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            // Use the two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Add evenly 30m spaced points ahead of the starting reference
          for (int i = 1; i <= NUM_WAYPOINTS; i++)
          {
            double new_s = car_s + i * WAYPOINT_SPACING;
            double new_d = LANE_WIDTH / 2 + LANE_WIDTH * lane;
            vector<double> next_wp = getXY(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]);
          }

          // shift car reference angle to 0 degrees
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create a spline
          tk::spline s;
          // Set (x,y) points to the spline (i.e. fits a spline to those points)
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // Calculate how to break up spline points so that we travel at desired velocity
          double target_y = s(WAYPOINT_SPACING);
          double target_dist = sqrt(WAYPOINT_SPACING * WAYPOINT_SPACING + target_y * target_y); // this is the d in the diagram
          double x_add_on = 0.0;                                                                // Related to the transformation (starting at zero)
          // Fill up the rest of path planner after filling it with previous points, will always output PLANNER_SIZE points
          for (int i = 1; i <= PLANNER_SIZE - previous_path_x.size(); i++)
          {
            // Reduce speed if too close, add if no longer close
            // Try and Switch Lanes, else slow down
            if (shouldSlowDown)
            {
              ref_vel -= MAX_DELTA_SPEED;
            }
            else if (ref_vel < SPEED_LIMIT)
            {
              ref_vel += MAX_DELTA_SPEED;
            }
            double N = (target_dist / (INTERVAL * ref_vel / MS_TO_MPH));
            double x_point = x_add_on + WAYPOINT_SPACING / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate x, y back to normal
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
