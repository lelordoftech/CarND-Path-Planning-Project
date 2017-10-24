#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <vector>
#include "json.hpp"
#include "spline.h"
#include "helpers.h"
#include "ptg.h"

using namespace std;

// Udacity
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

// Me
void vehicle_map(std::map<int8_t, Vehicle>* predictions,
                  double ref_s, double pre_path_size,
                  nlohmann::basic_json<>* sensor_fusion);
int8_t find_nearest_vehicle(std::map<int8_t, Vehicle>* predictions,
                            int8_t ref_lane, double ref_s);
bool find_best_traj(struct trajectory** best_traj, int8_t* ref_lane,
                    int8_t vehicle_id, std::map<int8_t, Vehicle>* predictions,
                    Ptg* ptg, struct state* ref_state);
void* createSpline(double pre_x, double pre_y,
                          double ref_x, double ref_y, double ref_s,
                          double ref_yaw, int8_t ref_lane,
                          vector<double> map_waypoints_s,
                          vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y);
void* createSpline(double pre_x, double pre_y,
                          double ref_x, double ref_y, double ref_yaw,
                          vector<double> map_waypoints_s,
                          vector<double> map_waypoints_x,
                          vector<double> map_waypoints_y,
                          struct trajectory* traj);
void planPath(vector<double>* next_x_vals, vector<double>* next_y_vals,
              double ref_x, double ref_y, double ref_s, double ref_yaw,
              double ref_vel, double pre_path_size, void* spline);
#endif // ALGORITHM_H
