#ifndef HELPERS_H
#define HELPERS_H

#include <iostream> // null
#include <map>

#define SIGMA_T                   2.0
#define MAX_JERK                  10 // m/s/s/s
#define MAX_ACCEL                 10 // m/s/s
#define EXPECTED_JERK_IN_ONE_SEC  2 // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC   1 // m/s
#define SPEED_LIMIT               49.5
#define VEHICLE_RADIUS            1.5 // model vehicle as circle to simplify collision detection

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
};

struct state
{
  struct model s;
  struct model d;
  state(){};
  state(double new_s, double new_s_dot, double new_s_ddot, double new_d, double new_d_dot, double new_d_ddot)
  {
    s = model(new_s, new_s_dot, new_s_ddot);
    d = model(new_d, new_d_dot, new_d_ddot);
  };
  void add(struct model new_s, struct model new_d)
  {
    s.m += new_s.m;
    s.m_dot += new_s.m_dot;
    s.m_ddot += new_s.m_ddot;
    d.m += new_d.m;
    d.m_dot += new_d.m_dot;
    d.m_ddot += new_d.m_ddot;
  };
  void add(struct state new_state)
  {
    s.m += new_state.s.m;
    s.m_dot += new_state.s.m_dot;
    s.m_ddot += new_state.s.m_ddot;
    d.m += new_state.d.m;
    d.m_dot += new_state.d.m_dot;
    d.m_ddot += new_state.d.m_ddot;
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
  Vehicle(struct state start);
  ~Vehicle();
  struct state state_in(double t);
};

double calculate(double* coeffs, double t, uint8_t N=6);
void plot_vehicle(double T, const char* name, const char* plot_type, Vehicle* vehicle = NULL, bool isShow = false);
void plot_trajectory(double* s_coeffs, double* d_coeffs, double T, const char* name, const char* plot_type, bool isShow = false);
void show_trajectory();
double logistic(double x);
void differentiate(double* out_coeffs, double* coefficients, uint8_t N=5);
void get_f_and_N_derivatives(double* out_coeffs, double* coeffs, uint8_t N, double T);
double nearest_approach_to_any_vehicle(struct trajectory traj, std::map<int8_t, Vehicle> vehicles);
double nearest_approach(struct trajectory traj, Vehicle vehicle);

#endif // HELPERS_H
