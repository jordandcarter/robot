/*
  Pid.h - Library for pid controller http://en.wikipedia.org/wiki/PID_controller
  Created by Jordan D Carter, January 5, 2015.
  Released into the public domain.
*/

#include "Pid.h"

Pid::Pid(double Kp, double Ki, double Kd)
{
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _proportionalTerm = 0;
  _integralTerm = 0;
  _derivativeTerm = 0;
  _previousError = 0;
}

void Pid::loop(double dt, double target, double current)
{
  _error = target - current;
  _integralTerm += _error*dt;
  _derivativeTerm = (_error - _previousError)/dt;
  _output = _Kp*_proportionalTerm + _Ki*_integralTerm + _Kd*_derivativeTerm;
  _previousError = _error;
}

double Pid::output()
{
  return _output;
}
