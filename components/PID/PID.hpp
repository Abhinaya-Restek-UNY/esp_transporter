#pragma once
#include "PresistentConfig.hpp"

class PID {
public:
  PID(double Kp, double Ki, double Kd, double delta_t, double limMin,
      double limMax, double limMinInt, double limMaxInt);

  PID();

  double update(double setpoint, double measurement, double measured_velocity);

  struct Config {
    double Kp;
    double Ki;
    double Kd;
  };

  void save_config();

  Config *config;

private:
  double delta_t;

  double limMin, limMax;
  double limMinInt, limMaxInt;

  double integrator;
  double prevError;
  double differentiator;
  double out;

  PresistentConfig<Config> config_handler;
};
