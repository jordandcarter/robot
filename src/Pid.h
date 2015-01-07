/*
  Pid.h - Library for pid controller http://en.wikipedia.org/wiki/PID_controller
  Created by Jordan D Carter, January 5, 2015.
  Released into the public domain.
*/
#ifndef Pid_h
#define Pid_h

class Pid
{
  public:
    Pid();
    Pid(double Kp, double Ki, double Kd);
    void loop(unsigned long dt, double target, double current);
    double output();
  private:
    double _target;
    double _current;
    double _error;
    double _previousError;
    double _Kp;
    double _Ki;
    double _Kd;
    double _proportionalTerm;
    double _integralTerm;
    double _derivativeTerm;
    double _output;
};

#endif

