#include "ptg.h"

/*
 * Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).
 * 
 * @param: ref_state - [s, s_dot, s_ddot, d, d_dot, d_ddot]
 * @param: target_vehicle - id of leading vehicle (int) which can be used to retrieve
      that vehicle from the "predictions" dictionary. This is the vehicle that 
      we are setting our trajectory relative to.
 * @param: delta - a length 6 array indicating the offset we are aiming for between us
      and the target_vehicle.
      So if at time 5 the target vehicle will be at [100, 10, 0, 0, 0, 0] 
      and delta is [-10, 0, 0, 4, 0, 0], 
      then our goal state for t = 5 will be [90, 10, 0, 4, 0, 0]. 
      This would correspond to a goal of 
      "follow 10 meters behind and 4 meters to the right of target vehicle"
 * @param: T - the desired time at which we will be at the goal (relative to now as t=0)
 * @param: predictions - dictionary of {v_id : vehicle }. Each vehicle has a method 
      vehicle.state_in(time) which returns a length 6 array giving that vehicle's
      expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.
 * 
 * @return: (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
      best_d gives coefficients for d(t) and best_t gives duration associated w/ 
      this trajectory.
 */
struct trajectory* Ptg::PTG(struct state* ref_state, int8_t target_vehicle, struct state* delta, double T, std::map<int8_t, Vehicle>* predictions)
{
  struct trajectory* best_traj = NULL;

  if (target_vehicle > -1)
  {
    Vehicle* target = &predictions->at(target_vehicle);

    // generate alternative goals
    std::vector<struct goal> all_goals;
    double timestep = 0.2;
    double t = T - 4 * timestep;
    while (t <= T + 4 * timestep) // loop 8 times = 8 traj
    {
      struct state target_state = target->state_in(t);
      target_state.add(&delta->s, &delta->d);

      struct model goal_s = target_state.s;
      struct model goal_d = target_state.d;
      struct goal goals;
      goals.s = goal_s;
      goals.d = goal_d;
      goals.t = t;

      all_goals.push_back(goals);

      t += timestep;
    }

    // find best trajectory
    std::map<double, struct trajectory*> cost_traj;
    for (std::vector<struct goal>::iterator it=all_goals.begin(); it!=all_goals.end(); ++it)
    {
      double cost = 0;
      struct trajectory* traj = new trajectory();

      struct model s_goal = it->s;
      struct model d_goal = it->d;
      double t = it->t;
      JMT(traj->s_coeffs, &ref_state->s, &s_goal, t);
      JMT(traj->d_coeffs, &ref_state->d, &d_goal, t);
      traj->T = t;

      cost = calculate_cost(traj, target_vehicle, delta, T, predictions);

      cost_traj[cost] = traj; // Sort by cost
#ifdef VISUAL_DEBUG
      graph->plot_trajectory(ref_state->s.m, traj);
#endif // VISUAL_DEBUG
    }

    if (cost_traj.size() > 0)
    {
      best_traj = cost_traj.begin()->second; // Minimum cost
    }
  }
  else
  {
    printf("[ERROR] Can not find vehicle with id: %d\n", target_vehicle);
  }

  return best_traj;
}

/*
 * Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).
 * 
 * @param: ref_state - [s, s_dot, s_ddot, d, d_dot, d_ddot]
 * @param: target_state - a length 6 array indicating the offset we are aiming for between us and the target_state.
 * @param: T - the desired time at which we will be at the goal (relative to now as t=0)
 * @param: predictions - dictionary of {v_id : vehicle }.
 * 
 * @return: (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
      best_d gives coefficients for d(t) and best_t gives duration associated w/ 
      this trajectory.
 */
struct trajectory* Ptg::PTG(struct state* ref_state, struct state* target_state, double T, std::map<int8_t, Vehicle>* predictions)
{
  struct trajectory* best_traj = NULL;
  int8_t target_vehicle = -1;

  // generate alternative goals
  std::vector<struct goal> all_goals;
  double timestep = 0.2;
  double t = T - 4 * timestep;
  while (t <= T + 4 * timestep) // loop 8 times = 8 traj
  {
    struct model goal_s = target_state->s;
    struct model goal_d = target_state->d;
    struct goal goals;
    goals.s = goal_s;
    goals.d = goal_d;
    goals.t = t;

    all_goals.push_back(goals);

    t += timestep;
  }

  // find best trajectory
  std::map<double, struct trajectory*> cost_traj;
  for (std::vector<struct goal>::iterator it=all_goals.begin(); it!=all_goals.end(); ++it)
  {
    double cost = 0;
    struct trajectory* traj = new trajectory();

    struct model s_goal = it->s;
    struct model d_goal = it->d;
    double t = it->t;
    JMT(traj->s_coeffs, &ref_state->s, &s_goal, t);
    JMT(traj->d_coeffs, &ref_state->d, &d_goal, t);
    traj->T = t;

    cost = calculate_cost(traj, target_vehicle, target_state, T, predictions);

    cost_traj[cost] = traj; // Sort by cost
#ifdef VISUAL_DEBUG
    graph->plot_trajectory(ref_state->s.m, traj);
#endif // VISUAL_DEBUG
  }

  if (cost_traj.size() > 0)
  {
    best_traj = cost_traj.begin()->second; // Minimum cost
  }

  return best_traj;
}

/*
 * Return a total cost of this trajectory
 */
double Ptg::calculate_cost(struct trajectory* traj, int8_t target_vehicle, struct state* delta, double goal_t, std::map<int8_t, Vehicle>* predictions, bool verbose)
{
  double cost = 0;
  double new_cost = 0;

  for (weight_func_map::iterator it=weighted_cost_functions.begin(); it!=weighted_cost_functions.end(); ++it)
  {
    new_cost = it->second.begin()->first * (*it->second.begin()->second)(traj, target_vehicle, delta, goal_t, predictions, verbose);
    cost += new_cost;
  }

  return cost;
}

/*
 * Calculates Jerk Minimizing Trajectory for start, end and T.
 */
void Ptg::JMT(double* coeffs, struct model* start, struct model* end, double T)
{
  coeffs[0] = start->m;
  coeffs[1] = start->m_dot;
  coeffs[2] = start->m_ddot / 2.0;
  double c_0 = coeffs[0] + coeffs[1] * T + coeffs[2] * T * T;
  double c_1 = coeffs[1] + 2 * coeffs[2] * T;
  double c_2 = 2 * coeffs[2];

  // A*X = B
  MatrixXd A = MatrixXd::Zero(3, 3);
  A <<  T*T*T, T*T*T*T, T*T*T*T*T,
        3*T*T, 4*T*T*T, 5*T*T*T*T,
          6*T,  12*T*T,  20*T*T*T;

  VectorXd B = VectorXd::Zero(3);
  B <<  end->m - c_0,
        end->m_dot - c_1,
        end->m_ddot - c_2;

  VectorXd X = A.inverse() * B;
  coeffs[3] = X[0];
  coeffs[4] = X[1];
  coeffs[5] = X[2];
}
