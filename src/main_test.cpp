#include "helpers.h"
#include "ptg.h"

int main()
{
  struct state start(0, 10, 0, 0, 0, 0);
  Vehicle vehicle = Vehicle(start);
  std::map<int8_t, Vehicle> predictions;
  predictions[0] = vehicle;
  int8_t target = 0;
  struct state delta;
  struct model start_s(10, 10, 0);
  struct model start_d(4, 0, 0);
  double T = 5.0;
  Ptg* ptg = new Ptg();
  struct trajectory best = ptg->PTG(start_s, start_d, target, delta, T, predictions);

  return 0;
}
