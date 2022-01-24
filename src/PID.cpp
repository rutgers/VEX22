#include "PID.hpp"
#include <cstdio>
#include <cstdlib>

//A general purpose PID class that allows for the creation of arbitrary PID controller


//Constructor
PID::PID(double& p, double& i, double& d, double& e_t)
{
    setCoeffs(p, i, d, e_t);
    update_target(0);
}

//Sets the coefficents for the controller
void PID::setCoeffs(double p, double i, double d, double e_t)
{
    kp = p;
    ki = i;
    kd = d;
    err_thresh = e_t;
}

//Methods for returning each of the coefficents
double PID::get_kp() { return kp; }
double PID::get_ki() { return ki; }
double PID::get_kd() { return kd; }
double PID::get_target() { return target; }

//Updates the target of the controller, setting other values to defaults
void PID::update_target(double new_target)
{
  target = new_target;
  last_err = new_target;
  error_sum = 0;
}

//Updates the current reading of the PID controller, and return the output required to approach the target
double PID::update(double measure, double dt)
{
  double err = target - measure;
  printf("err: %f\n", err);
  error_sum += err*dt;
  double output = kp*err + ki*error_sum + kd*(err-last_err)/dt;
  last_err = err;

  return output;
}

//Check if the position of the PID controller is close to the target position
bool PID::check_arrived()
{
  return abs(last_err) < err_thresh;
}
