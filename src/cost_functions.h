#ifndef COST_FUNC_H
#define COST_FUNC_H

#include <vector>
#include <map>
#include "helpers.h" // struct type

double time_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double s_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double d_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double collision_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double buffer_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double stays_on_road_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double exceeds_speed_limit_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double efficiency_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double total_accel_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double max_accel_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double max_jerk_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);
double total_jerk_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose=false);

#endif // COST_FUNC_H
