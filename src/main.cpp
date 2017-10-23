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

using namespace std;

// for convenience
using json = nlohmann::json;

Graph* graph = Graph::getInstance();

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

/*
 * Create vehicle map in the future from sensor_fusion
 * 
 * @param: ref_s (m) - position s of our Car in the future
 * @param: pre_path_size - size of pre-path, which our Car does not move on
 * @param: sensor_fusion - json information
 * 
 * @output: predictions - map of id and vehicle
 */
void vehicle_map(std::map<int8_t, Vehicle>* predictions,
                double ref_s, double pre_path_size,
                nlohmann::basic_json<>* sensor_fusion)
{
  for (uint8_t i = 0; i < sensor_fusion->size(); i++)
  {
    // id, car_x, car_y, car_x_vel, car_y_vel, car_s, car_d
    int8_t check_id = (*sensor_fusion)[i][0];
    double check_vx = (*sensor_fusion)[i][3];
    double check_vy = (*sensor_fusion)[i][4];
    double check_s = (*sensor_fusion)[i][5];
    double check_d = (*sensor_fusion)[i][6];
    double check_speed = sqrt(check_vx*check_vx + check_vy*check_vy);
    // vehicle in the future pre_path_size frame
    // 0.02 seconds / frame
    check_s += (double)pre_path_size*0.02*check_speed;

    // Just add which vehicles in the front of vehicle in range 2*DIST_PLANNING
    // and in behide 4*VEHICLE_RADIUS
    // and only in 3 lanes
    if (check_s > ref_s-4*VEHICLE_RADIUS && 
        check_s < ref_s+2*DIST_PLANNING &&
        check_d >= 0 &&
        check_d <= 12)
    {
      struct state start(check_s, check_speed, 0, check_d, 0, 0);
      Vehicle vehicle = Vehicle(&start);
      (*predictions)[check_id] = vehicle;
    }
    else
    {
      // Ignore others car
    }
  }
}

/*
 * Find the nearest vehiclen in ref_lane, ref_s
 * 
 * @param: predictions - map of id and vehicle
 * @param: ref_lane - lane of our Car in the future
 * @param: ref_s (m) - position s of our Car in the future
 * 
 * @output: nearest_id : id of the nearest vehicle
 */
int8_t find_nearest_vehicle(std::map<int8_t, Vehicle>* predictions,
                            int8_t ref_lane, double ref_s)
{
  int8_t vehicle_id = -1;
  double min_s = ref_s+DIST_PLANNING;

  for (std::map<int8_t, Vehicle>::iterator it=predictions->begin(); it!=predictions->end(); ++it)
  {
    double check_s = it->second.state_in(0).s.m;
    double check_d = it->second.state_in(0).d.m;
    // lane width is 4, so we need to check vehicle in range +-2 of center lane
    if (check_d > (double)(2+4*ref_lane - 2) && check_d < (double)(2+4*ref_lane + 2))
    {
      if (check_s <= min_s)
      {
        vehicle_id = it->first;
        min_s = check_s;
      }
      else
      {
        // Do nothing
      }
    }
    else
    {
      // Do not check vehicle in other lane
    }
  }

  return vehicle_id;
}

/*
 * Find the best trajactory
 * In our lane -> consider switch lane or keep lane
 * 
 * @param: vehicle_id - id of the nearest vehicle
 * @param: predictions - map of id and vehicle
 * @param: ptg - pointer to Ptg object
 * @param: ref_state - state of our Car in the future
 * 
 * @output: best_traj - pointer to the best trajectory
 * @output: ref_lane - lane of our Car in the future
 * @output: is_following - flag to know when we must slow down
 */
bool find_best_traj(struct trajectory** best_traj, int8_t* ref_lane,
                    int8_t vehicle_id, std::map<int8_t, Vehicle>* predictions,
                    Ptg* ptg, struct state* ref_state)
{
  bool is_following = false;
  int8_t lane = *ref_lane;

  if (vehicle_id == -1)
  {
    // Go straight
    // We create a virtual vehicle with same state with us,
    // and try to keep follow this virtual vehicle
    double target_s = ref_state->s.m+DIST_PLANNING;
    double target_s_dot = SPEED_LIMIT/2.24;
    double target_d = (2+lane*4);
    struct state target(target_s, target_s_dot, 0, target_d, 0, 0);
    *best_traj = ptg->PTG(ref_state, &target, TIME_PLANNING, predictions);
  }
  else
  {
    Vehicle vehicle = predictions->at(vehicle_id);
    double check_s = vehicle.state_in(0).s.m;
    double check_d = vehicle.state_in(0).d.m;
    struct state delta;

    if (check_s <= ref_state->s.m+DIST_PLANNING &&
              check_s >= ref_state->s.m+2*VEHICLE_RADIUS)
    {
      // In dangerous region
      // Switch lane
      std::map<double, std::map<int8_t, struct trajectory*>> map_cost; // [cost : lane : traj]

      // Check all cases possible
      for (int8_t i = 0; i < 3; i++)
      {
        if (i != lane)
        {
          // In other lanes
          double delta_d = 4*(i-lane) - (2+lane*4-check_d);
          delta.set(0, 0, 0, delta_d, 0, 0); // switch lane to center
        }
        else
        {
          // In our lane
          // If we can not switch lane, try to following and wait the chance to switch
          double delta_d = (2+lane*4) - check_d;
          delta.set(-DIST_PLANNING/2.0, 0, 0, delta_d, 0, 0); // planning following DIST_PLANNING/2.0 in the center
        }
        struct trajectory* best = ptg->PTG(ref_state, vehicle_id, &delta, TIME_PLANNING, predictions);
        if (best != NULL)
        {
          double cost = ptg->calculate_cost(best, vehicle_id, &delta, TIME_PLANNING, predictions);
          map_cost[cost][i] = best; // Sort by cost to find minimum cost easier
        }
      }

      // Find best choice
      if (map_cost.size() > 0)
      {
        lane = map_cost.begin()->second.begin()->first;
        *best_traj = map_cost.begin()->second.begin()->second;
        if (lane == *ref_lane)
        {
          // Choose case following
          is_following = true;
        }
        else
        {
          // Choose case switch lane
          *ref_lane = lane;
        }
      }
    }
    else
    {
      // check_s > ref_state->s.m+DIST_PLANNING     -> In safe region
      // check_s < ref_state->s.m+2*VEHICLE_RADIUS  -> Case vehicle in behide us
      // Go straight
      // We create a virtual vehicle with same state with us,
      // and try to keep follow this virtual vehicle
      double target_s = ref_state->s.m+DIST_PLANNING;
      double target_s_dot = SPEED_LIMIT/2.24;
      double target_d = (2+lane*4);
      struct state target(target_s, target_s_dot, 0, target_d, 0, 0);
      *best_traj = ptg->PTG(ref_state, &target, TIME_PLANNING, predictions);
    }
  }

  return is_following;
}

/*
 * Create spline in XY Car cordinator
 * base on an constant distance in the future
 * 
 * @param: pre_x (m)
 * @param: pre_y (m)
 * @param: ref_x (m)
 * @param: ref_y (m)
 * @param: ref_s (m)
 * @param: ref_yaw (rad)
 * @param: ref_lane
 * @param: map_waypoints_s
 * @param: map_waypoints_x
 * @param: map_waypoints_y
 * 
 * @output: s - spline
 */
tk::spline* createSpline(double pre_x, double pre_y,
                          double ref_x, double ref_y, double ref_s,
                          double ref_yaw, int8_t ref_lane,
                          vector<double> map_waypoints_s,
                          vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y)
{
  printf("[MAIN][WARNING] Create spline base on an constant distance in the future\n");
  vector<double> pts_x;
  vector<double> pts_y;

  // Make point for spline calculation
  // 5 points
  // pre_x : ref_x : x1 : x2 : x3
  // pre_y : ref_y : y1 : y2 : y3
  pts_x.push_back(pre_x);
  pts_y.push_back(pre_y);
  pts_x.push_back(ref_x);
  pts_y.push_back(ref_y);

  for (uint8_t i = 1; i < 4; i++)
  {
    double next_s = ref_s + (double)i*DIST_PLANNING;
    double next_d = (double)(2 + 4*ref_lane);
    vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    pts_x.push_back(next_xy[0]);
    pts_y.push_back(next_xy[1]);
  }

  // Transformation to car coordinates
  for (uint8_t i = 0; i < pts_x.size(); i++)
  {
    // translation
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;
    // rotation clockwise psi
    pts_x[i] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
    pts_y[i] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
  }

  // Remove wrong data before apply spline
  for (uint8_t i = 1; i < pts_x.size();)
  {
    if (pts_x[i] > pts_x[i-1])
    {
      i++;
    }
    else
    {
      pts_x.erase(pts_x.begin()+i);
      pts_y.erase(pts_y.begin()+i);
    }
  }

  // Apply spline
  tk::spline* s = NULL;
  if (pts_x.size() > 2)
  {
    s = new tk::spline();
    s->set_points(pts_x, pts_y);
  }
  else
  {
    printf("[MAIN][ERROR] Cannot generate the next 3 point for spline generation\n");
  }

  return s;
}

/*
 * Create spline in XY Car cordinator
 * base on best trajectory
 * 
 * @param: pre_x (m)
 * @param: pre_y (m)
 * @param: ref_x (m)
 * @param: ref_y (m)
 * @param: ref_yaw (rad)
 * @param: map_waypoints_s
 * @param: map_waypoints_x
 * @param: map_waypoints_y
 * @param: traj
 * 
 * @output: s - spline
 */
tk::spline* createSpline(double pre_x, double pre_y,
                          double ref_x, double ref_y, double ref_yaw,
                          vector<double> map_waypoints_s,
                          vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y,
                          struct trajectory* traj)
{
  vector<double> pts_x;
  vector<double> pts_y;

  // Make point for spline calculation
  // 5 points
  // pre_x : ref_x : x1 : x2 : x3
  // pre_y : ref_y : y1 : y2 : y3
  pts_x.push_back(pre_x);
  pts_y.push_back(pre_y);
  pts_x.push_back(ref_x);
  pts_y.push_back(ref_y);

  double time_step = traj->T/2.0;
  for (uint8_t i = 1; i < 3; i++)
  {
    double next_s = calculate(traj->s_coeffs, i*time_step);
    double next_d = calculate(traj->d_coeffs, i*time_step);
    vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    pts_x.push_back(next_xy[0]);
    pts_y.push_back(next_xy[1]);
  }

  // Add more points
  double next_s = calculate(traj->s_coeffs, traj->T) + DIST_PLANNING;
  double next_d = calculate(traj->d_coeffs, traj->T);
  vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  pts_x.push_back(next_xy[0]);
  pts_y.push_back(next_xy[1]);

  // Transformation to car coordinates
  for (uint8_t i = 0; i < pts_x.size(); i++)
  {
    // translation
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;
    // rotation clockwise psi
    pts_x[i] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
    pts_y[i] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
  }

  // Remove wrong data before apply spline
  for (uint8_t i = 1; i < pts_x.size();)
  {
    if (pts_x[i] > pts_x[i-1])
    {
      i++;
    }
    else
    {
      pts_x.erase(pts_x.begin()+i);
      pts_y.erase(pts_y.begin()+i);
    }
  }

  // Apply spline
  tk::spline* s = NULL;
  if (pts_x.size() > 2)
  {
    s = new tk::spline();
    s->set_points(pts_x, pts_y);
  }
  else
  {
    printf("[MAIN][ERROR] Cannot generate the next 3 point for spline generation\n");
  }

  return s;
}

/*
 * Create path for moving
 * 
 * @param: ref_x (m)
 * @param: ref_y (m)
 * @param: ref_s (m)
 * @param: ref_yaw (rad)
 * @param: ref_vel (m/s)
 * @param: pre_path_size
 * @param: s
 * 
 * @output: next_x_vals
 * @output: next_y_vals
 */
void planPath(vector<double>* next_x_vals, vector<double>* next_y_vals,
              double ref_x, double ref_y, double ref_s, double ref_yaw,
              double ref_vel, double pre_path_size, tk::spline s)
{
  double target_x = DIST_PLANNING/2.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double ref_vel_step = ref_vel*0.02;
  double N = target_dist/ref_vel_step; // split target_dist to N pieces
  double ref_x_step = target_x/N;
  double x_ref = 0.0;
  double y_ref = 0.0;

  for (uint8_t i = 1; i <= 50-pre_path_size; i++)
  {
    x_ref = i*ref_x_step;
    y_ref = s(x_ref);

    // Transformation to global coordinates
    // rotation counter-clockwise psi
    double x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    double y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
    // translation
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals->push_back(x_point);
    next_y_vals->push_back(y_point);
  }
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
                &ref_lane, &ref_vel, &best_traj, &ptg](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            s = createSpline(pre_x, pre_y,
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
            s = createSpline(pre_x, pre_y,
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
                    ref_vel, pre_path_size, *s);

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
