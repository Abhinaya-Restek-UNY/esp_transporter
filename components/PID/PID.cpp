#include "PID.hpp"
#include <algorithm>
#include <cmath>
#include <stdio.h>

PID::PID(double Kp, double Ki, double Kd, double delta_t_ms, double limMin,
         double limMax, double limMinInt, double limMaxInt)
    : delta_t(delta_t_ms / 1000.0), limMin(limMin), limMax(limMax),
      limMinInt(limMinInt), limMaxInt(limMaxInt), integrator(0.0),
      prevError(0.0), differentiator(0.0), out(0.0),
      config_handler("pid", {0, 0, 0}) {
  this->config = &config_handler.value;
}

PID::PID()
    : delta_t(10 / 1000.0), limMin(-1.0), limMax(1), limMinInt(-0.3),
      limMaxInt(), integrator(0.0), prevError(0.0), differentiator(0.0),
      out(0.0), config_handler("pid", {0, 0, 0}) {
  this->config = &config_handler.value;
}

// Added 'measured_velocity' as the third parameter
double PID::update(double setpoint, double measurement,
                   double measured_velocity) {

  double error = measurement - setpoint;

  while (error > M_PI)
    error -= 2.0 * M_PI;
  while (error < -M_PI)
    error += 2.0 * M_PI;
  // Proportional
  double proportional = this->config->Kp * error;

  // Integral (Trapezoidal integration) with anti-windup clamping
  integrator +=
      0.5 * this->config->Ki * this->delta_t * (error + this->prevError);
  integrator = std::max(limMinInt, std::min(integrator, limMaxInt));

  // Derivative (Directly from your filtered Gyro reading!)
  // We keep it negative because this is Derivative on Measurement
  differentiator = -this->config->Kd * measured_velocity;

  // Total Output
  out = proportional + integrator + differentiator;
  out = std::max(limMin, std::min(out, limMax));

  // State update
  prevError = error;

  // Note: prevMeasurement is completely gone. We don't need it anymore!

  return out;
}

void PID::save_config() { this->config_handler.save(); };
