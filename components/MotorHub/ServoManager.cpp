#include "ServoManager.hpp"
#include "Panic.h"
#include "driver/mcpwm_cmpr.h"

Servo::Servo(mcpwm_cmpr_handle_t cmp, uint16_t max_angle, uint32_t min_tick,
             uint32_t max_tick)
    : max_angle(max_angle), cmp(cmp), min_tick(min_tick) {
  this->scale = (double)(max_tick - min_tick) / max_angle;
};

void Servo::set_angle(float angle) {
  if (angle > this->max_angle) {
    angle = this->max_angle;
  } else if (angle < 0.0f) {
    angle = 0.0f;
  }

  ERR_CHECK(mcpwm_comparator_set_compare_value(
      this->cmp, this->min_tick + (uint32_t)(angle * this->scale)));
};

Servo *ServoManager::create_servo(gpio_num_t PWM, uint16_t max_angle,
                                  float min_pulse_ms, float max_pulse_ms) {

  if (this->total_mcpwm >= MAX_PWM) {
    return nullptr;
  }

  float _period = 1000.0f / this->frequency;

  uint32_t min_ticks = (min_pulse_ms * this->period) / _period;
  uint32_t max_ticks = (max_pulse_ms * this->period) / _period;

  Servo *new_servo =
      new Servo(create_pwm(PWM), max_angle, min_ticks, max_ticks);

  push_node(&this->last_serv, new_servo);

  this->total_mcpwm++;
  return new_servo;
};
