#include "PID.hpp"
#include <algorithm>

PID::PID(double Kp, double Ki, double Kd, double delta_t_ms, double limMin,
         double limMax, double limMinInt, double limMaxInt)
    : Kp(Kp), Ki(Ki), Kd(Kd), delta_t(delta_t_ms / 1000.0), limMin(limMin),
      limMax(limMax), limMinInt(limMinInt), limMaxInt(limMaxInt),
      integrator(0.0), prevError(0.0), differentiator(0.0), out(0.0) {}

// Added 'measured_velocity' as the third parameter
double PID::update(double setpoint, double measurement,
                   double measured_velocity) {
  double error = setpoint - measurement;

  // Proportional
  double proportional = Kp * error;

  // Integral (Trapezoidal integration) with anti-windup clamping
  integrator += 0.5 * this->Ki * this->delta_t * (error + this->prevError);
  integrator = std::max(limMinInt, std::min(integrator, limMaxInt));

  // Derivative (Directly from your filtered Gyro reading!)
  // We keep it negative because this is Derivative on Measurement
  differentiator = -Kd * measured_velocity;

  // Total Output
  out = proportional + integrator + differentiator;
  out = std::max(limMin, std::min(out, limMax));

  // State update
  prevError = error;

  // Note: prevMeasurement is completely gone. We don't need it anymore!

  return out;
}
