#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:
  PID(double Kp, double Ki, double Kd, double delta_t, double limMin,
      double limMax, double limMinInt, double limMaxInt);

  double update(double setpoint, double measurement, double measured_velocity);

  void set_param(double Kp, double Ki, double Kd);

private:
  // Gains
  double Kp, Ki, Kd;

  // Timing & Filter
  double delta_t;

  // Limits
  double limMin, limMax;
  double limMinInt, limMaxInt;

  // States
  double integrator;
  double prevError;
  double differentiator;
  double out;
};

#endif
