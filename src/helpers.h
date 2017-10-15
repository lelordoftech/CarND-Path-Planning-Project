#ifndef HELPERS_H
#define HELPERS_H

#include <iostream> // null

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

double calculate(double* coeffs, double t);
void plot_trajectory(double* s_coeffs, double* d_coeffs, double T, Vehicle* vehicle = NULL, bool isShow = false);
void show_trajectory();

#endif // HELPERS_H