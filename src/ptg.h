#ifndef PTG_FUNC_H
#define PTG_FUNC_H

#include <map>
#include "Eigen-3.3/Eigen/Dense" // VectorXd, MatrixXd
#include "helpers.h" // struct type
#include "cost_functions.h" // cost functions

using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef double (*function)(struct trajectory* traj, int8_t target_vehicle, struct state* delta, double T, std::map<int8_t, Vehicle>* predictions, bool verbose);
typedef std::map<int8_t, std::map<int8_t, function>> weight_func_map;

class Ptg
{
private:
  weight_func_map weighted_cost_functions;
  Graph* graph;
public:
  Ptg()
  {
    weighted_cost_functions[0][2] = &s_diff_cost;
    weighted_cost_functions[1][2] = &d_diff_cost;
    weighted_cost_functions[2][100] = &efficiency_cost;
    weighted_cost_functions[3][1] = &time_diff_cost;
    weighted_cost_functions[4][10] = &exceeds_speed_limit_cost;
    weighted_cost_functions[5][1] = &max_jerk_cost;
    weighted_cost_functions[6][1] = &total_jerk_cost;
    weighted_cost_functions[7][1] = &max_accel_cost;
    weighted_cost_functions[8][1] = &total_accel_cost;
    weighted_cost_functions[9][10] = &collision_cost;
    weighted_cost_functions[10][1] = &buffer_cost;
    //weighted_cost_functions[11][0] = &stays_on_road_cost;
    //weighted_cost_functions[12][0] = &safety_change_lane_cost;

    graph = Graph::getInstance();
  };
  ~Ptg(){};

  struct trajectory* PTG(struct state* ref_state, int8_t target_vehicle, struct state* delta, double T, std::map<int8_t, Vehicle>* predictions);
  struct trajectory* PTG(struct state* ref_state, struct state* target_state, double T, std::map<int8_t, Vehicle>* predictions);
  double calculate_cost(struct trajectory* traj, int8_t target_vehicle, struct state* delta, double goal_t, std::map<int8_t, Vehicle>* predictions, bool verbose=false);

private:
  void JMT(double* coeffs, struct model* start, struct model* end, double T);
};

#endif // PTG_FUNC_H
