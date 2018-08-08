#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "Behavior_planning/spline.h"
#include "Behavior_planning/road.h"
#include "Behavior_planning/vehicle.h"
#include "helper_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s
  // and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  load_Waypoints (map_waypoints_x, map_waypoints_y, map_waypoints_s,
                  map_waypoints_dx, map_waypoints_dy);

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  /*
  Behavior_planning configuration
  */
  //impacts default behavior for most states
  int SPEED_LIMIT         = 49;

  //all traffic in lane (besides ego) follow these speeds
  vector<int> LANE_SPEEDS = {49, 49, 49};

  // At each timestep, ego can set acceleration to value between
  //-MAX_ACCEL and MAX_ACCEL
  int MAX_ACCEL           = 10;

  // s value and lane number of goal.
  vector<int> GOAL        = {(int) max_s, 1};

  Road road = Road(SPEED_LIMIT, LANE_SPEEDS);

  //configuration data:  target speed, speed limit, num_lanes,
  //                     goal_s, goal_lane, max_acceleration
	int num_lanes          = LANE_SPEEDS.size();
	vector<int> ego_config = {SPEED_LIMIT, num_lanes, GOAL[0],
		                        GOAL[1], MAX_ACCEL, SPEED_LIMIT};

  //start in lane 1 cause we set lane goal is 1
  int lane = 1;
  //have a reference velocity to target
  double ref_vel = 0.0; //max: 49.5 mph

  // start at lane, s = 0 (assume), and configuration: ego_config
	//road.add_ego(lane, 0, ego_config);
  road.add_ego(lane, 0, ref_vel, ego_config);

  h.onMessage([&road, &lane, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x     = j[1]["x"];
          	double car_y     = j[1]["y"];
          	double car_s     = j[1]["s"];
          	double car_d     = j[1]["d"];
          	double car_yaw   = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

            // Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars
            // on the same side of the road. vector<vector<double>>
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            if (prev_size > 0) car_s = end_path_s;

            road.ego_localization(car_s);

            road.add_vehicles_surrounding(sensor_fusion, prev_size);

            road.behavior_planning();

            Vehicle ego = road.get_ego();

            std::cout << " [EGO] state: " << ego.state
                      << " lane:"  << ego.lane
                      << " velocity: " << ego.v
                      << " ego_s: " << ego.s << " car_s: " << car_s
                      << std::endl;

            if (car_d < (2 + 4*lane +2) && car_d > (2 + 4*lane -2)) lane = ego.lane;

            if (ref_vel > ego.v)
               ref_vel -= .224 * 2 ;

            else if (ref_vel < ego.v)
               ref_vel += .224 * 2;

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we will interoplate these waypoints with a spline and
            // fill it in with more points that control speed

            Waypoints wp(prev_size, lane, car_x, car_y, car_yaw, car_s,
                         map_waypoints_s, map_waypoints_x, map_waypoints_y,
                         previous_path_x, previous_path_y);

            wp.spaced_waypoints_generator ();
            wp.detailed_waypoints_generator(ref_vel);

            json msgJson;
            msgJson["next_x"] = wp.next_x_vals;
            msgJson["next_y"] = wp.next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
  // program doesn't compile :-(

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
