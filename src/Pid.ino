/*
  Pid.h - Library for pid controller http://en.wikipedia.org/wiki/PID_controller
  Created by Jordan D Carter, January 5, 2015.
  Released into the public domain.
*/

#include "Pid.h"
#include "Arduino.h"

Pid::Pid()
{
  _Kp = 0.1;
  _Ki = 0;
  _Kd = 0;
  _proportionalTerm = 0;
  _integralTerm = 0;
  _derivativeTerm = 0;
  _previousError = 0;
}

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
  if (dt <= 0){
    dt = 0.000000001;
  }
  _proportionalTerm = target - current;
  if(abs(_integralTerm + (_proportionalTerm*dt)) < 4000){
    _integralTerm += _proportionalTerm*dt;
  }
  _derivativeTerm = (_proportionalTerm - _previousError)/dt;
  _output = _Kp*_proportionalTerm + _Ki*_integralTerm + _Kd*_derivativeTerm;
  _previousError = _proportionalTerm;
}

double Pid::output()
{
  return _output;
}
