#include <string.h> // memcpy
#include <cmath> // pow
#include <vector>

#include "helpers.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

Vehicle::Vehicle()
{
  // Do nothing
}

Vehicle::Vehicle(struct state start)
{
  memcpy(&start_state, &start, sizeof(struct state));
}

Vehicle::~Vehicle()
{
  // Do nothing
}

/*
 * Return state of vehicle in time t
 */
struct state Vehicle::state_in(double t)
{
  struct model s = start_state.s;
  struct model d = start_state.d;
  struct state returnValue(s.m + (s.m_dot * t) + s.m_ddot * t*t / 2.0,
                            s.m_dot + s.m_ddot * t,
                            s.m_ddot,
                            d.m + (d.m_dot * t) + d.m_ddot * t*t / 2.0,
                            d.m_dot + d.m_ddot * t,
                            d.m_ddot);

  return returnValue;
}

/*
 * Calculate f(t) = coeffs(t)
 */
double calculate(double* coeffs, double t, uint8_t N)
{
  double returnValue = 0.0;
  for (uint8_t i = 0; i < N; i++)
  {
    returnValue += coeffs[i] * pow(t, i);
  }
  return returnValue;
}

/*
 * Plot vehicle into graph
 */
void plot_vehicle(double T, const char* name, const char* plot_type, Vehicle* vehicle, bool isShow)
{
  std::vector<double> X2;
  std::vector<double> Y2;

  for (double t = 0.0; t < T; t+=0.25)
  {
    if (vehicle != NULL)
    {
      struct state cur_state = vehicle->state_in(t);
      X2.push_back(cur_state.s.m);
      Y2.push_back(cur_state.d.m);
    }
  }
  if (vehicle != NULL)
  {
    plt::named_plot(name, X2, Y2, plot_type);
  }
  if (isShow == true)
  {
    show_trajectory();
  }
}

/*
 * Plot trajectory into graph
 */
void plot_trajectory(double* s_coeffs, double* d_coeffs, double T, const char* name, const char* plot_type, bool isShow)
{
  std::vector<double> X;
  std::vector<double> Y;

  for (double t = 0.0; t < T; t+=0.25)
  {
    X.push_back(calculate(s_coeffs, t));
    Y.push_back(calculate(d_coeffs, t));
  }
  if (name != "")
  {
    plt::named_plot(name, X, Y, plot_type);
  }
  else
  {
    plt::plot(X, Y, plot_type);
  }
  if (isShow == true)
  {
    show_trajectory();
  }
}

/*
 * Show trajectory graph
 */
void show_trajectory()
{
  // Enable legend.
  plt::legend();
  // Label
  plt::xlabel("S");
  plt::ylabel("D");
  // Save image file
  plt::save("../output_images/show_trajectory.png");
  // Show
  plt::show();
}

/*
 * A function that returns a value between 0 and 1 for x in the 
 *  range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

 *  Useful for cost functions.
 */
double logistic(double x)
{
  return 2.0 / (1 + exp(-x)) - 1.0;
}

/*
 * Calculates the derivative of a polynomial and returns
 *  the corresponding coefficients.
 */
void differentiate(double* out_coeffs, double* coefficients, uint8_t N)
{
  for (uint8_t i = 0; i < N; i++)
  {
    out_coeffs[i] = (i+1) * coefficients[i+1];
  }
}

/*
 * Return f and N derivatives
 */
void get_f_and_N_derivatives(double* out_coeffs, double* coeffs, uint8_t N, double T)
{
  out_coeffs[0] = calculate(coeffs, T);
  for (uint8_t i = 0; i < N; i++)
  {
    double new_cos[6-i-1];
    differentiate(new_cos, coeffs, 6-i-1);
    out_coeffs[i+1] = calculate(coeffs, T, 6-i-1);
  }
}

/*
 * Calculates the closest distance to any vehicle during a trajectory.
 */
double nearest_approach_to_any_vehicle(struct trajectory traj, std::map<int8_t, Vehicle> vehicles)
{
  double closest = 999999;
  for (std::map<int8_t, Vehicle>::iterator it=vehicles.begin(); it!=vehicles.end(); ++it)
  {
    Vehicle v = it->second;
    double d = nearest_approach(traj, v);
    if (d < closest)
    {
      closest = d;
    }
  }
  return closest;
}

/*
 * Calculates the closest distance to vehicle during a trajectory.
 */
double nearest_approach(struct trajectory traj, Vehicle vehicle)
{
  double closest = 999999;
  double* s_ = traj.s_coeffs;
  double* d_ = traj.d_coeffs;
  double T = traj.T;
  for (uint8_t i = 0; i < 10; i++)
  {
    double t = double(i) / 100 * T;
    double cur_s = calculate(s_, t);
    double cur_d = calculate(d_, t);
    struct state curr_state = vehicle.state_in(t);
    double targ_s = curr_state.s.m;
    double targ_d = curr_state.d.m;
    double dist = sqrt(pow(cur_s-targ_s, 2) + pow(cur_d-targ_d, 2));
    if (dist < closest)
    {
      closest = dist;
    }
  }
  return closest;
}
