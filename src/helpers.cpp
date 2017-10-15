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

double calculate(double* coeffs, double t)
{
  double returnValue = 0.0;
  for (uint8_t i = 0; i < 6; i++)
  {
    returnValue += coeffs[i] * pow(t, i);
  }
  return returnValue;
}

void plot_trajectory(double* s_coeffs, double* d_coeffs, double T, Vehicle* vehicle, bool isShow)
{
  std::vector<double> X;
  std::vector<double> Y;
  std::vector<double> X2;
  std::vector<double> Y2;

  double t = 0;
  while (t <= T+0.01)
  {
    X.push_back(calculate(s_coeffs, t));
    Y.push_back(calculate(d_coeffs, t));
    if (vehicle != NULL)
    {
      struct state cur_state = vehicle->state_in(t);
      X2.push_back(cur_state.s.m);
      Y2.push_back(cur_state.d.m);
    }
    t += 0.25;
  }

  plt::plot(X, Y, "b--");
  if (vehicle != NULL)
  {
    plt::plot(X2, Y2, "r--");
  }
  if (isShow == true)
  {
    show_trajectory();
  }
}

void show_trajectory()
{
  // Enable legend.
  plt::legend();
  // Save image file
  plt::save("../output_images/show_trajectory.png");
  // Show
  plt::show();
}