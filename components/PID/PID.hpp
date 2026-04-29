#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <algorithm>

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, double T, 
                  double tau = 0.02, 
                  double limMin = -1024.0, double limMax = 1024.0, 
                  double limMinInt = -300.0, double limMaxInt = 300.0);

    double update(double setpoint, double measurement);

private:
    // Gains
    double Kp, Ki, Kd;
    
    // Timing & Filter
    double T, tau;

    // Limits
    double limMin, limMax;
    double limMinInt, limMaxInt;

    // States
    double integrator;
    double prevError;
    double differentiator;
    double prevMeasurement;
    double out;
};

#endif