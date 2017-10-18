#include "helpers.h"
#include "ptg.h"

int main()
{
  // [s, s_dot, s_ddot, d, d_dot, d_ddot]
  struct state start(5, 15, 0, 6, 0, 0); // go straight with constant velocity 15m/s
  Vehicle vehicle = Vehicle(start); // leading vehicle
  struct state start2(10, 15, 0, 10, 0, 0);
  Vehicle vehicle2 = Vehicle(start2); // leading vehicle 2
  std::map<int8_t, Vehicle> predictions;
  int8_t target_id_1 = 0;
  int8_t target_id_2 = 1;
  predictions[target_id_1] = vehicle;
  predictions[target_id_2] = vehicle2;

  struct state delta(-10, 0, 0, 1, 0, 0); // planning behide 10m, in the right 1m

  struct model start_s(0, 22, 0); // current [s, s_dot, s_ddot]
  struct model start_d(2, 0, 0); // current [d, d_dot, d_ddot]
  double T = 1.0; // planning in 1 seconds

  Ptg* ptg = new Ptg();
  Graph* graph = Graph::getInstance();
  struct trajectory best = ptg->PTG(start_s, start_d, target_id_1, delta, T, predictions);
  double cost1 = ptg->calculate_cost(best, target_id_1, delta, TIME_PLANNING, predictions);
  struct trajectory best2 = ptg->PTG(start_s, start_d, target_id_2, delta, T, predictions);
  double cost2 = ptg->calculate_cost(best2, target_id_2, delta, TIME_PLANNING, predictions);
  printf("Cost 1: %f\n", cost1);
  printf("Cost 2: %f\n", cost2);

#ifdef VISUAL_DEBUG
  graph->plot_trajectory(0, best.s_coeffs, best.d_coeffs, best.T, Scalar(0, 0, 255));
  graph->plot_trajectory(0, best2.s_coeffs, best2.d_coeffs, best2.T, Scalar(0, 0, 255));
  graph->plot_vehicle(0, best.T, Scalar(0, 0, 255), &vehicle);
  graph->plot_vehicle(0, best2.T, Scalar(0, 0, 255), &vehicle2);
  while (true)
  {
    uint8_t k = graph->show_trajectory();
    if (k == 27) // Wait for ESC key to exit
    {
      break;
    }
  }
  destroyAllWindows();
#endif // VISUAL_DEBUG

  delete ptg;
  ptg = NULL;
  delete graph;
  graph = NULL;

  return 0;
}
