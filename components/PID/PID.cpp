#include "PIDController.h"

PIDController::PIDController(double Kp, double Ki, double Kd, double T, 
                             double tau, double limMin, double limMax, 
                             double limMinInt, double limMaxInt)
    : Kp(Kp), Ki(Ki), Kd(Kd), T(T), tau(tau), 
      limMin(limMin), limMax(limMax), 
      limMinInt(limMinInt), limMaxInt(limMaxInt),
      integrator(0.0), prevError(0.0), differentiator(0.0), 
      prevMeasurement(0.0), out(0.0) {}

double PIDController::update(double setpoint, double measurement) {
    double error = setpoint - measurement;

    // Proportional
    double proportional = Kp * error;

    // Integral (Trapezoidal integration) with anti-windup clamping
    integrator += 0.5 * Ki * T * (error + prevError);
    integrator = std::max(limMinInt, std::min(integrator, limMaxInt));

    // Derivative (Band-limited differentiator)
    differentiator = -(2.0 * Kd * (measurement - prevMeasurement) 
                     + (2.0 * tau - T) * differentiator) 
                     / (2.0 * tau + T);

    // Total Output
    out = proportional + integrator + differentiator;
    out = std::max(limMin, std::min(out, limMax));

    // State update
    prevError = error;
    prevMeasurement = measurement;

    return out;
}