#include "Gripper.hpp"
#include "freertos/idf_additions.h"
#include <cmath>
#include <cstdio>

#define MAX_CLAW_ANGLE 95
#define MIN_CLAW_ANGLE 70

#define MAX_LIFT_ANGLE 85
#define MIN_LIFT_ANGLE 0

Gripper::Gripper(Servo *claw, Servo *lifter, uint32_t debounce)
    : claw(claw), lifter(lifter) {
  this->set_claw(true);
  this->set_lifter(0);
}

void Gripper::set_claw(bool gripping) {
  this->claw->set_angle(gripping ? MIN_CLAW_ANGLE : MAX_CLAW_ANGLE);
}
void Gripper::set_lifter(int16_t throttlebrake) {
  static double half = (MAX_LIFT_ANGLE - MIN_LIFT_ANGLE) * 0.6;
  if (throttlebrake > 10) {

    this->lifter->set_angle(half * pow(1.0 - throttlebrake / 1024.0, 3));
  } else {
    this->lifter->set_angle(MAX_LIFT_ANGLE);
  }
}

uint32_t Gripper::get_time() { return esp_timer_get_time() / 1000; }

Gripper::Gripper() {
  // NOTE: YES DOES NOTHING.

};
