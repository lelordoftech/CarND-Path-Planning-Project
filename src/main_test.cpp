#include "helpers.h"
#include "ptg.h"

int main()
{
  // [s, s_dot, s_ddot, d, d_dot, d_ddot]
  struct state start(0, 10, 0, 0, 0, 0); // go straight with constant velocity 10m/s
  Vehicle vehicle = Vehicle(start); // leading vehicle
  struct state start2(0, 10, 0, 3, 0.2, -0.01);
  Vehicle vehicle2 = Vehicle(start2); // leading vehicle 2
  std::map<int8_t, Vehicle> predictions;
  predictions[0] = vehicle;
  predictions[1] = vehicle2;
  int8_t target = 0;
  int8_t target2 = 1;
  struct state delta(-10, 0, 0, -1, 0, 0); // planning behide 10m, in the right 1m
  struct model start_s(10, 10, 0); // current s
  struct model start_d(2, 0, 0); // current d
  double T = 10.0; // planning in 10 seconds

  Ptg* ptg = new Ptg();
  struct trajectory best = ptg->PTG(start_s, start_d, target, delta, T, predictions);
  plot_trajectory(best.s_coeffs, best.d_coeffs, best.T, "best traj", "g--");

  struct trajectory best2 = ptg->PTG(start_s, start_d, target2, delta, T, predictions);
  plot_trajectory(best2.s_coeffs, best2.d_coeffs, best2.T, "best traj 2", "g--");

  plot_vehicle(best.T, "vehicle 1", "r--", &vehicle);
  plot_vehicle(best2.T, "vehicle 2", "y--", &vehicle2);

  show_trajectory();

  return 0;
}
