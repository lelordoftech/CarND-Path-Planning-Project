#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>

#include "algorithm.h"

int main()
{
  uint16_t counter = 0;

  // Our car's information
  Ptg* ptg = new Ptg();
  Graph* graph = Graph::getInstance();
  struct trajectory* best_traj = NULL;
  int8_t ref_lane = -1;
  double ref_s = 0.0;
  double ref_d = 6.5;
  struct state ref_state(ref_s, 22, 0, ref_d, 0, 0); // current [s, s_dot, s_ddot, d, d_dot, d_ddot]

  // Other vehicle in the road
  std::map<int8_t, Vehicle> predictions;

  // Vehicle map to read from
  string map_file_ = "../data/vehicle_map.csv";

  while (true)
  {
#ifdef VISUAL_DEBUG
    graph->initGraph();
#endif // VISUAL_DEBUG

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    int8_t target_id = -1;
    while (getline(in_map_, line)) {
      istringstream iss(line);
      double s;
      double s_dot;
      double s_ddot;
      double d;
      double d_dot;
      double d_ddot;
      iss >> s;
      iss >> s_dot;
      iss >> s_ddot;
      iss >> d;
      iss >> d_dot;
      iss >> d_ddot;
      if (target_id == -1)
      {
        ref_state.set(s, s_dot, s_ddot, d, d_dot, d_ddot);
        ref_s = s;
        ref_d = d;
      }
      else
      {
        struct state vehicle_state(s, s_dot, s_ddot, d, d_dot, d_ddot);
        Vehicle vehicle = Vehicle(&vehicle_state); // leading vehicle
        predictions[target_id] = vehicle;
      }
      target_id++;
    }
    ref_lane = ref_d/4;

#ifdef VISUAL_DEBUG
    for (std::map<int8_t, Vehicle>::iterator it=predictions.begin(); it!=predictions.end(); ++it)
    {
      Vehicle vehicle = it->second;
      graph->plot_vehicle(ref_s, TIME_PLANNING, Scalar(0, 0, 255), &vehicle);
    }
#endif // VISUAL_DEBUG

    int8_t nearest_id = find_nearest_vehicle(&predictions,
                                              ref_lane, ref_s);

    bool is_following = find_best_traj(&best_traj, &ref_lane,
                                        nearest_id, &predictions,
                                        ptg, &ref_state);
    printf("[%4d][TEST MAIN] nearest_id=%d : is_following=%s : ref_lane=%d\n",
            counter,
            nearest_id,
            is_following ? "true" : "false",
            ref_lane);

#ifdef VISUAL_DEBUG
    // Plot our trajectory
    if (best_traj != NULL)
    {
      graph->plot_trajectory(ref_s, best_traj, Scalar(200, 0, 255));
    }

    Vehicle vehicle = Vehicle(&ref_state);
    graph->plot_vehicle(ref_s, TIME_PLANNING, Scalar(0, 255, 0), &vehicle);

    uint8_t k = graph->show_trajectory("../output_images/test_trajectory.png");
    if (k == 27) // Wait for ESC key to exit
    {
      break;
    }
#endif // VISUAL_DEBUG

    // Reset data
    while (!predictions.empty())
    {
      predictions.erase(predictions.begin());
    }

    counter++;
    this_thread::sleep_for(chrono::milliseconds(1000));
  }
  destroyAllWindows();

  delete ptg;
  ptg = NULL;
  delete graph;
  graph = NULL;

  return 0;
}
