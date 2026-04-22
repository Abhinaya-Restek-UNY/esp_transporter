#pragma once
#include "driver/gpio.h"
#include "driver/mcpwm_cmpr.h"

class MotorHub;

class Motor {
  friend class MotorHub;

protected:
  Motor(mcpwm_cmpr_handle_t cmp, gpio_num_t A, gpio_num_t B);

public:
  void set_direction(int16_t direction);

private:
  gpio_num_t A;
  gpio_num_t B;
  mcpwm_cmpr_handle_t cmp;
};
