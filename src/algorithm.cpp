#include "algorithm.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
void* createSpline(double pre_x, double pre_y,
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

  return (void*)s;
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
void* createSpline(double pre_x, double pre_y,
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

  return (void*)s;
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
              double ref_vel, double pre_path_size, void* spline)
{
  tk::spline s = *(tk::spline*)spline;
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
