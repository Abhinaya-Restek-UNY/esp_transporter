#include "Gripper.hpp"
#include "freertos/idf_additions.h"
#include <cstdio>

#define MAX_CLAW_ANGLE 100
#define MIN_CLAW_ANGLE 60

#define MAX_LIFT_ANGLE 90
#define MIN_LIFT_ANGLE 0

Gripper::Gripper(Servo *claw, Servo *lifter, uint32_t debounce)
    : claw(claw), lifter(lifter) {
  this->set_claw(true);
  this->set_lifter(1024);
}

void Gripper::set_claw(bool gripping) {
  this->claw->set_angle(gripping ? MIN_CLAW_ANGLE : MAX_CLAW_ANGLE);
}
void Gripper::set_lifter(int16_t throttlebrake) {
  this->lifter->set_angle(MIN_LIFT_ANGLE + (MAX_LIFT_ANGLE - MIN_LIFT_ANGLE) *
                                               throttlebrake / 1024.0);
}

uint32_t Gripper::get_time() { return esp_timer_get_time() / 1000; }

Gripper::Gripper() {
  // NOTE: YES DOES NOTHING.

};
