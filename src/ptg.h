#ifndef PTG_FUNC_H
#define PTG_FUNC_H

#include <map>
#include "Eigen-3.3/Eigen/Dense" // VectorXd, MatrixXd
#include "helpers.h" // struct type
#include "cost_functions.h" // cost functions

using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef double (*function)(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose);
typedef std::map<int8_t, std::map<int8_t, function>> weight_func_map;

class Ptg
{
private:
  weight_func_map weighted_cost_functions;
  Graph* graph;
public:
  Ptg()
  {
    weighted_cost_functions[0][1] = &time_diff_cost;
    weighted_cost_functions[1][1] = &s_diff_cost;
    weighted_cost_functions[2][1] = &d_diff_cost;
    weighted_cost_functions[3][1] = &efficiency_cost;
    weighted_cost_functions[4][1] = &max_jerk_cost;
    weighted_cost_functions[5][1] = &total_jerk_cost;
    weighted_cost_functions[6][1] = &collision_cost;
    weighted_cost_functions[7][1] = &buffer_cost;
    weighted_cost_functions[8][1] = &max_accel_cost;
    weighted_cost_functions[9][1] = &total_accel_cost;

    graph = Graph::getInstance();
  };
  ~Ptg(){};

  struct trajectory PTG(struct model start_s, struct model start_d, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions);
  double calculate_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double goal_t, std::map<int8_t, Vehicle> predictions, bool verbose=false);

private:
  struct state perturb_goal(struct model goal_s, struct model goal_d);
  void JMT(double* coeffs, struct model start, struct model end, double T);
};

#endif // PTG_FUNC_H
