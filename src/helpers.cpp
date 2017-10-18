#include <string.h> // memcpy
#include <cmath> // pow
#include <vector>

#include "helpers.h"

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

#ifdef VISUAL_DEBUG
Graph* Graph::g_instance = NULL;

Graph::Graph()
{
  initGraph();
}

Graph::~Graph()
{
  //
}

/*
 * Init graph
 */
void Graph::initGraph()
{
  // Create black empty images
  g_image = Mat::zeros(440, 320, CV_8UC3);
  // Draw 3 lanes
  line(g_image, Point(40, 0), Point(40, 440), Scalar(110, 220, 0), 2, 8);
  line(g_image, Point(120, 0), Point(120, 440), Scalar(110, 220, 0), 2, 8);
  line(g_image, Point(200, 0), Point(200, 440), Scalar(110, 220, 0), 2, 8);
  line(g_image, Point(280, 0), Point(280, 440), Scalar(110, 220, 0), 2, 8);
}

/*
 * Generate trajectory graph into g_image
 */
void Graph::gen_trajectory(double car_s, std::vector<double>& X, std::vector<double>& Y, Scalar color)
{
  // Draw graph
  for (int i = 0; i < X.size()-1; i++)
  {
    line(g_image, Point(40+X[i]*20, 440 - (Y[i]-car_s)*20), Point(40+X[i+1]*20, 440 - (Y[i+1]-car_s)*20), color, 1, 8);
  }
}

/*
 * Plot trajectory into graph
 */
void Graph::plot_trajectory(double car_s, double* s_coeffs, double* d_coeffs, double T, Scalar color, bool isShow)
{
  std::vector<double> S;
  std::vector<double> D;

  for (double t = 0.0; t < T; t+=0.02)
  {
    S.push_back(calculate(s_coeffs, t));
    D.push_back(calculate(d_coeffs, t));
  }
  gen_trajectory(car_s, D, S, color);
  if (isShow == true)
  {
    show_trajectory();
  }
}

/*
 * Plot vehicle into graph
 */
void Graph::plot_vehicle(double car_s, double T, Scalar color, Vehicle* vehicle, bool isShow)
{
  std::vector<double> S;
  std::vector<double> D;

  for (double t = 0.0; t < T; t+=0.02)
  {
    if (vehicle != NULL)
    {
      struct state cur_state = vehicle->state_in(t);
      S.push_back(cur_state.s.m);
      D.push_back(cur_state.d.m);
    }
  }
  if (vehicle != NULL)
  {
    gen_trajectory(car_s, D, S, color);
    circle(g_image, Point(40+D[0]*20, 440 - (S[0]-car_s)*20), 10, color, 1, 8);
  }
  if (isShow == true)
  {
    show_trajectory();
  }
}

/*
 * Show trajectory graph
 */
uint8_t Graph::show_trajectory()
{
  // Save image file
  char img_name[] = "../output_images/show_trajectory.png";
  imwrite(img_name, g_image);
  // Show
  imshow("Path Planning", g_image);
  return waitKey(1);
}
#endif // VISUAL_DEBUG
