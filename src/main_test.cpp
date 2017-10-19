#include "helpers.h"
#include "ptg.h"

int main()
{
  // [s, s_dot, s_ddot, d, d_dot, d_ddot]
  struct state start1(20, 10, 0, 2.5, 0, 0);
  Vehicle vehicle1 = Vehicle(start1); // leading vehicle
  struct state start2(1, 10, 0, 6, 0, 0); // go straight with constant velocity 15m/s
  Vehicle vehicle2 = Vehicle(start2); // leading vehicle 2
  struct state start3(25, 10, 0, 10, 0, 0); // go straight with constant velocity 15m/s
  Vehicle vehicle3 = Vehicle(start3); // leading vehicle 3
  std::map<int8_t, Vehicle> predictions;
  int8_t target_id_1 = 0;
  int8_t target_id_2 = 1;
  int8_t target_id_3 = 2;
  predictions[target_id_1] = vehicle1;
  predictions[target_id_2] = vehicle2;
  predictions[target_id_3] = vehicle3;

  struct state delta(-5, 0, 0, 0, 0, 0); // planning behide 5m
  struct state delta2(0, 0, 0, 4, 0, 0); // planning in the right 4m
  struct state delta3(0, 0, 0, 8, 0, 0); // planning in the right 8m

  struct model start_s(0, 22, 0); // current [s, s_dot, s_ddot]
  struct model start_d(2, 0, 0); // current [d, d_dot, d_ddot]
  double T = TIME_PLANNING; // planning in 2 seconds

  Ptg* ptg = new Ptg();
  Graph* graph = Graph::getInstance();
  struct trajectory best1 = ptg->PTG(start_s, start_d, target_id_1, delta, T, predictions);
  double cost1 = ptg->calculate_cost(best1, target_id_1, delta, TIME_PLANNING, predictions);
  struct trajectory best2 = ptg->PTG(start_s, start_d, target_id_1, delta2, T, predictions);
  double cost2 = ptg->calculate_cost(best2, target_id_1, delta2, TIME_PLANNING, predictions);
  struct trajectory best3 = ptg->PTG(start_s, start_d, target_id_1, delta3, T, predictions);
  double cost3 = ptg->calculate_cost(best3, target_id_1, delta3, TIME_PLANNING, predictions);
  printf("Cost 1: %f\n", cost1);
  printf("Cost 2: %f\n", cost2);
  printf("Cost 3: %f\n", cost3);

#ifdef VISUAL_DEBUG
  graph->plot_trajectory(0, best1.s_coeffs, best1.d_coeffs, best1.T, Scalar(0, 255, 0));  
  graph->plot_trajectory(0, best2.s_coeffs, best2.d_coeffs, best2.T, Scalar(0, 255, 0));  
  graph->plot_trajectory(0, best3.s_coeffs, best3.d_coeffs, best3.T, Scalar(0, 255, 0));  
  graph->plot_vehicle(0, best1.T, Scalar(0, 0, 255), &vehicle1);
  graph->plot_vehicle(0, best3.T, Scalar(0, 0, 255), &vehicle2);
  graph->plot_vehicle(0, best3.T, Scalar(0, 0, 255), &vehicle3);
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
