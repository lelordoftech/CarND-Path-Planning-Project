#ifndef HELPERS_H
#define HELPERS_H

#include <iostream> // null
#include <map>

#ifdef VISUAL_DEBUG
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#endif // VISUAL_DEBUG

#define SIGMA_T                   2.0 // s
#define MAX_JERK                  10.0 // m/s/s/s
#define MAX_ACCEL                 10.0 // m/s/s
#define EXPECTED_JERK_IN_ONE_SEC  2.0 // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC   1.0 // m/s
#define SPEED_LIMIT               49.5 // mph ~ 22m/s
#define VEHICLE_RADIUS            2.0 // m: model vehicle as circle to simplify collision detection
// max speed is 49.5 mph = 49.5/2.24=22m/s
#define TIME_PLANNING             2.0 // s
#define DIST_PLANNING             49.5/2.24*TIME_PLANNING

#ifdef VISUAL_DEBUG
#define SCALE_WIDTH               20
#define SCALE_HEIGHT              5
#define GRAPH_WIDTH               16*SCALE_WIDTH // 16m, scale 20px
#define GRAPH_HEIGHT              (2*DIST_PLANNING+12+4*VEHICLE_RADIUS)*SCALE_HEIGHT // 88m+12m+6m scale 5px
#define PLOT_IMAGE_PATH           "../output_images/show_trajectory.png"
#endif

const double SIGMA_S[3] = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
const double SIGMA_D[3] = {1.0, 1.0, 1.0};

struct trajectory
{
  double s_coeffs[6];
  double d_coeffs[6];
  double T;
};

struct model
{
  double m;
  double m_dot;
  double m_ddot;
  model()
  {
    m = 0;
    m_dot = 0;
    m_ddot = 0;
  };
  model(double new_m, double new_m_dot, double new_m_ddot)
  {
    m = new_m;
    m_dot = new_m_dot;
    m_ddot = new_m_ddot;
  };
  void set(double new_m, double new_m_dot, double new_m_ddot)
  {
    m = new_m;
    m_dot = new_m_dot;
    m_ddot = new_m_ddot;
  };
};

struct state
{
  struct model s;
  struct model d;
  state(){};
  state(double new_s, double new_s_dot, double new_s_ddot, double new_d, double new_d_dot, double new_d_ddot)
  {
    s.set(new_s, new_s_dot, new_s_ddot);
    d.set(new_d, new_d_dot, new_d_ddot);
  };
  void set(double new_s, double new_s_dot, double new_s_ddot, double new_d, double new_d_dot, double new_d_ddot)
  {
    s.set(new_s, new_s_dot, new_s_ddot);
    d.set(new_d, new_d_dot, new_d_ddot);
  };
  void add(struct model* new_s, struct model* new_d)
  {
    s.m += new_s->m;
    s.m_dot += new_s->m_dot;
    s.m_ddot += new_s->m_ddot;
    d.m += new_d->m;
    d.m_dot += new_d->m_dot;
    d.m_ddot += new_d->m_ddot;
  };
  void add(struct state* new_state)
  {
    s.m += new_state->s.m;
    s.m_dot += new_state->s.m_dot;
    s.m_ddot += new_state->s.m_ddot;
    d.m += new_state->d.m;
    d.m_dot += new_state->d.m_dot;
    d.m_ddot += new_state->d.m_ddot;
  };
};

struct goal
{
  struct model s;
  struct model d;
  double t;
};

class Vehicle
{
private:
  struct state start_state;
public:
  Vehicle();
  Vehicle(struct state* start);
  ~Vehicle();
  struct state state_in(double t);
};

double calculate(double* coeffs, double t, uint8_t N=6);
double logistic(double x);
void differentiate(double* out_coeffs, double* coefficients, uint8_t N=5);
void get_f_and_N_derivatives(double* out_coeffs, double* coeffs, uint8_t N, double T);
double nearest_approach(struct trajectory* traj, Vehicle* vehicle);
double nearest_approach_to_any_vehicle(struct trajectory* traj, std::map<int8_t, Vehicle>* vehicles);

#ifdef VISUAL_DEBUG
class Graph
{
public:
  Graph();
  ~Graph();

  static Graph* getInstance()
  {
    if (g_instance == NULL)
    {
      g_instance = new Graph();
    }
    return g_instance;
  };

  void initGraph();
  void plot_trajectory(double car_s, struct trajectory* traj, Scalar color = Scalar(255, 0, 0), bool isShow = false);
  void plot_vehicle(double car_s, double T, Scalar color = Scalar(0, 255, 0), Vehicle* vehicle = NULL, bool isShow = false);
  void plot_vehicle(double car_s, std::vector<double>* X, std::vector<double>* Y, Scalar color, bool isShow = false);
  uint8_t show_trajectory(const char* img_name = PLOT_IMAGE_PATH);
private:
  Mat g_image;
  static Graph* g_instance;
  void gen_trajectory(double car_s, std::vector<double>* X, std::vector<double>* Y, Scalar color = Scalar(255, 0, 0));
#endif // VISUAL_DEBUG
};

#endif // HELPERS_H
