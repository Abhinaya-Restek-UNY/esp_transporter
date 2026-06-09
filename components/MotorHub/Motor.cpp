#include "Motor.hpp"
#include "Panic.h"
#include <cstdlib>

Motor::Motor(mcpwm_cmpr_handle_t cmp, gpio_num_t A, gpio_num_t B)
    : A(A), B(B), cmp(cmp) {

  gpio_set_direction(A, GPIO_MODE_OUTPUT);
  gpio_set_direction(B, GPIO_MODE_OUTPUT);

  this->set_direction(0);
};

void Motor::set_direction(int16_t direction) {
  if (direction == 0) {

    ERR_CHECK(mcpwm_comparator_set_compare_value(this->cmp, 0));
    gpio_set_level(this->A, 1);
    gpio_set_level(this->B, 1);
    return;
  }
  uint16_t speed = abs(direction);

  ERR_CHECK(mcpwm_comparator_set_compare_value(this->cmp, speed >> 4));

  if (direction > 0) {
    gpio_set_level(this->A, 1);
    gpio_set_level(this->B, 0);
  } else {
    gpio_set_level(this->A, 0);
    gpio_set_level(this->B, 1);
  }
};
