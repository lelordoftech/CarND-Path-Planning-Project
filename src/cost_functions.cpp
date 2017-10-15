#include "cost_functions.h"

double time_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double s_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double d_diff_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double collision_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double buffer_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double stays_on_road_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double exceeds_speed_limit_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double efficiency_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double total_accel_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double max_accel_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double max_jerk_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}

double total_jerk_cost(struct trajectory traj, int8_t target_vehicle, struct state delta, double T, std::map<int8_t, Vehicle> predictions, bool verbose)
{
  double cost = 0;

  if (verbose == true)
  {
    printf("%16s : %f\n", __func__, cost);
  }
  return cost;
}
