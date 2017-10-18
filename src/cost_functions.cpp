#include <algorithm> // std::min_element, std::max_element
#include "cost_functions.h"

/*
 * Penalizes trajectories that span a duration which is longer or 
 *  shorter than the duration requested.
 */
double time_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double t = traj.T;
  cost = logistic(double(abs(t-T)) / T);

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes trajectories whose s coordinate (and derivatives) 
 *  differ from the goal.
 */
double s_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* s = traj.s_coeffs;
  double t = traj.T;
  struct state target = predictions[target_vehicle].state_in(t);
  target.add(delta);
  double* s_targ = (double*)&target.s;
  double S[3];
  get_f_and_N_derivatives(S, s, 2, t);

  for (uint8_t i = 0; i < 3; i++)
  {
    double actual = S[i];
    double expected = s_targ[i];
    double sigma = SIGMA_S[i];
    double diff = double(abs(actual-expected));
    cost += logistic(diff/sigma);
  }

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes trajectories whose d coordinate (and derivatives) 
 *  differ from the goal.
 */
double d_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* d_coeffs = traj.d_coeffs;
  double t = traj.T;
  double d_dot_coeffs[5];
  differentiate(d_dot_coeffs, d_coeffs, 5);
  double d_ddot_coeffs[4];
  differentiate(d_ddot_coeffs, d_dot_coeffs, 4);
  double d = calculate(d_coeffs, t, 6);
  double d_dot = calculate(d_dot_coeffs, t, 5);
  double d_ddot = calculate(d_ddot_coeffs, t, 4);
  double D[3] = {d, d_dot, d_ddot};
  
  struct state target = predictions[target_vehicle].state_in(t);
  target.add(delta);
  double* d_targ = (double*)&target.d;

  for (uint8_t i = 0; i < 3; i++)
  {
    double actual = D[i];
    double expected = d_targ[i];
    double sigma = SIGMA_D[i];
    double diff = double(abs(actual-expected));
    cost += logistic(diff/sigma);
  }

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Binary cost function which penalizes collisions.
 */
double collision_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double nearest = nearest_approach_to_any_vehicle(traj, predictions);
  if (nearest < 2*VEHICLE_RADIUS)
  {
    cost = 1.0;
  }
  else
  {
    cost = 0.0;
  }

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes getting close to other vehicles.
 */
double buffer_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double nearest = nearest_approach_to_any_vehicle(traj, predictions);
  cost = logistic(2*VEHICLE_RADIUS / nearest);

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes stays on road.
 */
double stays_on_road_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  // TODO

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes exceeds speed limit.
 */
double exceeds_speed_limit_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  // TODO

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Rewards high average speeds.
 */
double efficiency_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* s = traj.s_coeffs;
  double t = traj.T;
  double avg_v = double(calculate(s, t)) / t;
  struct state curr_state = predictions[target_vehicle].state_in(t);
  double targ_s = curr_state.s.m;
  double targ_v = double(targ_s) / t;
  cost = logistic(2*double(targ_v - avg_v) / avg_v);

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes total accel.
 */
double total_accel_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* s = traj.s_coeffs;
  double* d = traj.d_coeffs;
  double t = traj.T;
  double s_dot[5];
  differentiate(s_dot, s, 5);
  double s_d_dot[4];
  differentiate(s_d_dot, s_dot, 4);
  double total_acc = 0.0;
  double dt = T / 100.0;
  double acc = 0.0;
  for (uint8_t i = 0; i < 100; i++)
  {
    t = dt * i;
    acc = calculate(s_d_dot, t, 4);
    total_acc += abs(acc*dt);
  }
  double acc_per_second = total_acc / T;
  cost = logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC );

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes max accel.
 */
double max_accel_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* s = traj.s_coeffs;
  double* d = traj.d_coeffs;
  double t = traj.T;
  double s_dot[5];
  differentiate(s_dot, s, 5);
  double s_d_dot[4];
  differentiate(s_d_dot, s_dot, 4);
  std::vector<double> all_accs;
  for (uint8_t i = 0; i < 100; i++)
  {
    all_accs.push_back(calculate(s_d_dot, T/100 * i, 4));
  }
  double max_acc = *std::max_element(all_accs.begin(), all_accs.end());
  if (abs(max_acc) > MAX_ACCEL)
  {
    cost = 1.0;
  }
  else
  {
    cost = 0.0;
  }

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes max jerk.
 */
double max_jerk_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* s = traj.s_coeffs;
  double* d = traj.d_coeffs;
  double t = traj.T;
  double s_dot[5];
  differentiate(s_dot, s, 5);
  double s_d_dot[4];
  differentiate(s_d_dot, s_dot, 4);
  double jerk[3];
  differentiate(jerk, s_d_dot, 3);
  std::vector<double> all_jerks;
  for (uint8_t i = 0; i < 100; i++)
  {
    all_jerks.push_back(calculate(jerk, T/100 * i, 4));
  }
  double max_jerk = *std::max_element(all_jerks.begin(), all_jerks.end());
  if (abs(max_jerk) > MAX_JERK)
  {
    cost = 1.0;
  }
  else
  {
    cost = 0.0;
  }

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

/*
 * Penalizes total jerk.
 */
double total_jerk_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0.0;
  double* s = traj.s_coeffs;
  double* d = traj.d_coeffs;
  double t = traj.T;
  double s_dot[5];
  differentiate(s_dot, s, 5);
  double s_d_dot[4];
  differentiate(s_d_dot, s_dot, 4);
  double jerk[3];
  differentiate(jerk, s_d_dot, 3);
  double total_jerk = 0.0;
  double dt = T / 100.0;
  double j = 0.0;
  for (uint8_t i = 0; i < 100; i++)
  {
    t = dt * i;
    j = calculate(jerk, t, 3);
    total_jerk += abs(j*dt);
  }
  double jerk_per_second = total_jerk / T;
  cost = logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC );

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}
