#include "Servo.hpp"
#include "Panic.h"
#include "driver/mcpwm_cmpr.h"

Servo::Servo(mcpwm_cmpr_handle_t cmp, int16_t max_angle, int16_t min_tick,
             int16_t max_tick)
    : max_angle(max_angle), cmp(cmp), min_tick(min_tick) {
  this->scale = (double)(max_tick - min_tick) / max_angle;
};

void Servo::set_angle(float angle) {
  if (angle > this->max_angle) {
    angle = this->max_angle;
  } else if (angle < 0) {
    angle = 0;
  }

  ERR_CHECK(mcpwm_comparator_set_compare_value(
      this->cmp, this->min_tick + angle * this->scale));
};
