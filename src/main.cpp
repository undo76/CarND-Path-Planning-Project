#include <uWS/uWS.h>
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "PathPlanifier.hpp"
#include "utils.hpp"
#include "json.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

SensorFusion build_sensor_fusion(json sensor_fusion) {
  SensorFusion ret;
  for (auto sf : sensor_fusion) {
    ret.id.push_back(sf[0].get<int>());
    ret.x.push_back(sf[1].get<double>());
    ret.y.push_back(sf[2].get<double>());
    ret.vx.push_back(sf[3].get<double>());
    ret.vy.push_back(sf[4].get<double>());
    ret.s.push_back(sf[5].get<double>());
    ret.d.push_back(sf[6].get<double>());
  }
  return ret;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  PathPlanifier path_planifier(map_waypoints_x, map_waypoints_y,
                               map_waypoints_s, map_waypoints_dx,
                               map_waypoints_dy);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy,
               &path_planifier](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          Car car;
          Path previous_path;

          car.x = j[1]["x"];
          car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = deg2rad(j[1]["yaw"]);
          car.speed = j[1]["speed"];

          // Previous path data given to the Planner
          previous_path.x = j[1]["previous_path_x"].get<vector<double>>();
          previous_path.y = j[1]["previous_path_y"].get<vector<double>>();
          // Previous path's end s and d values
          previous_path.end_s = j[1]["end_path_s"];
          previous_path.end_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road. [id, x, y, vx, vy, s, d]
          SensorFusion sensor_fusion =
              build_sensor_fusion(j[1]["sensor_fusion"]);

          json msgJson;

          auto next_path =
              path_planifier.update(car, previous_path, sensor_fusion);

          msgJson["next_x"] = next_path[0];
          msgJson["next_y"] = next_path[1];

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
