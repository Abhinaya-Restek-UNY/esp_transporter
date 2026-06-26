#include "Gripper.hpp"
#include "freertos/idf_additions.h"
#include <cmath>
#include <cstdio>

#define MAX_CLAW_ANGLE 90
#define MIN_CLAW_ANGLE 55

#define MAX_LIFT_ANGLE 78
#define MIN_LIFT_ANGLE 0

Gripper::Gripper(Servo *claw, Servo *lifter, float _max_claw,
                 const std::string &config_key)
    : claw(claw), lifter(lifter), max_angle(config_key, _max_claw) {
  this->set_claw(true);
  this->set_lifter(0);
}

void Gripper::set_claw(bool gripping) {
  this->claw->set_angle(gripping ? MIN_CLAW_ANGLE : this->max_angle.value);
}
void Gripper::set_lifter(int16_t throttlebrake) {
  static double half = (MAX_LIFT_ANGLE - MIN_LIFT_ANGLE) * .5;
  if (throttlebrake > 10) {
    int16_t val = throttlebrake < 256 ? 256 : throttlebrake;
    this->lifter->set_angle(half * (1.0 - pow(val / 1024.0, 3)));
  } else {
    this->lifter->set_angle(MAX_LIFT_ANGLE);
  }
}
