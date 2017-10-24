#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "spline.h"
#include <sys/time.h>
#include "helpers.h"
#include "ptg.h"
#include "algorithm.h"

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

/*
 * MAIN
 */
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

  int8_t ref_lane = 1; // we have 6 lanes: -2 -1 0 1 2
  double ref_vel = 0; // mph: start value should be 0
  struct trajectory* best_traj = NULL;
  Graph* graph = Graph::getInstance();
  Ptg* ptg = new Ptg();

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
                &ref_lane, &ref_vel, &best_traj, &graph, &ptg](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"]; // Convert to radian
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          // START IMPLEMENTATION
          struct timeval t_start, t_end;
          gettimeofday(&t_start, NULL);
#ifdef VISUAL_DEBUG
          graph->initGraph();
#endif // VISUAL_DEBUG

          // Step 1: Update state in the future
          // We will make planing path for the future, not for current
          double pre_x = 0.0;
          double pre_y = 0.0;
          double ref_x = 0.0;
          double ref_y = 0.0;
          double ref_s = 0.0;
          double ref_d = 0.0;
          double ref_yaw = 0.0;
          double ref_speed = car_speed/2.24; // convert mph to m/s
          uint8_t pre_path_size = previous_path_x.size();

          if (pre_path_size < 2)
          {
            ref_x = car_x;
            ref_y = car_y;
            pre_x = ref_x - cos(ref_yaw);
            pre_y = ref_y - sin(ref_yaw);
            ref_yaw = deg2rad(car_yaw); // Convert to radian
          }
          else
          {
            pre_x = previous_path_x[pre_path_size-2];
            pre_y = previous_path_y[pre_path_size-2];
            ref_x = previous_path_x[pre_path_size-1];
            ref_y = previous_path_y[pre_path_size-1];
            ref_yaw = atan2((ref_y-pre_y), (ref_x-pre_x));
          }

          if (pre_path_size > 0)
          {
            ref_s = end_path_s;
            ref_d = end_path_d;
          }
          else
          {
            ref_s = car_s;
            ref_d = car_d;
          }

          // Step 2: Make vehicle map
          std::map<int8_t, Vehicle> predictions;
          vehicle_map(&predictions, ref_s, pre_path_size, &sensor_fusion);

#ifdef VISUAL_DEBUG
          for (std::map<int8_t, Vehicle>::iterator it=predictions.begin(); it!=predictions.end(); ++it)
          {
            Vehicle vehicle = it->second;
            graph->plot_vehicle(ref_s, TIME_PLANNING, Scalar(0, 0, 255), &vehicle);
          }
#endif // VISUAL_DEBUG

          // Step 3: Find the nearest vehicle in the front of us
          int8_t nearest_id = find_nearest_vehicle(&predictions,
                                                    ref_lane, ref_s);

          // Step 4: Find the best trajectory, best ref_lane
          struct state ref_state(ref_s, ref_speed, 0, ref_d, 0, 0);

          // Try to come back our lanes 0,1,2 if we out of lanes
          if (ref_d < 0)
          {
            ref_lane = 0;
          }
          else if (ref_d > 12)
          {
            ref_lane = 2;
          }

          bool is_following = find_best_traj(&best_traj, &ref_lane,
                                              nearest_id, &predictions,
                                              ptg, &ref_state);

          // Step 5: Create path : 50 points = 1s
          for (uint8_t i = 0; i < pre_path_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Create spline
          tk::spline* s = NULL;
          if (best_traj != NULL)
          {
            s = (tk::spline*)createSpline(pre_x, pre_y,
                              ref_x, ref_y, ref_yaw,
                              map_waypoints_s,
                              map_waypoints_x,
                              map_waypoints_y,
                              best_traj);
          }
          else
          {
            printf("[MAIN][ERROR] Can not find traj\n");
          }

          if (s == NULL)
          {
            s = (tk::spline*)createSpline(pre_x, pre_y,
                              ref_x, ref_y, ref_s, ref_yaw,
                              ref_lane,
                              map_waypoints_s,
                              map_waypoints_x,
                              map_waypoints_y);
          }

          // Update velocity
          double target_vel = SPEED_LIMIT/2.24;
          double target_accel = MAX_ACCEL/2.0;
          if (nearest_id > -1 && is_following == true)
          {
            // Case following
            Vehicle vehicle = predictions.at(nearest_id);
            target_vel = vehicle.state_in(0).s.m_dot;
          }

          if (ref_vel < target_vel)
          {
            ref_vel += target_accel*0.02;
            if (ref_vel > target_vel)
            {
              ref_vel = target_vel;
            }
          }
          else if (ref_vel > target_vel)
          {
            ref_vel -= target_accel*0.02;
            if (ref_vel < target_vel)
            {
              ref_vel = target_vel;
            }
          }

          // Make planning path
          planPath(&next_x_vals, &next_y_vals,
                    ref_x, ref_y, ref_s, ref_yaw,
                    ref_vel, pre_path_size, (char*)s);

#ifdef VISUAL_DEBUG
          // Plot our trajectory
          if (best_traj != NULL)
          {
            graph->plot_trajectory(ref_s, best_traj, Scalar(200, 0, 255));
          }

          struct state start(ref_s, ref_speed, 0, ref_d, 0, 0);
          Vehicle vehicle = Vehicle(&start);
          graph->plot_vehicle(ref_s, TIME_PLANNING, Scalar(0, 255, 0), &vehicle);

          graph->show_trajectory();

          if (best_traj != NULL)
          {
            delete best_traj;
            best_traj = NULL;
            //return; // stop to debug

            gettimeofday(&t_end, NULL);
            uint16_t ms = (t_end.tv_sec-t_start.tv_sec) * 1000 +
                          (t_end.tv_usec-t_start.tv_usec) / 1000;
            printf("[MAIN][INFO] Time to process: %d\n", ms);
          }
          if (s != NULL)
          {
            delete s;
            s = NULL;
          }
#endif // VISUAL_DEBUG
          // END IMPLEMENTATION

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
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

  h.onConnection([&ref_lane, &ref_vel, &best_traj,
                  &h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    // Reset all value
    ref_lane = 1;
    ref_vel = 0;
    if (best_traj != NULL)
    {
      delete best_traj;
      best_traj = NULL;
    }
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
