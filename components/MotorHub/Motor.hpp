#pragma once
#include "driver/gpio.h"
#include "driver/mcpwm_cmpr.h"

class MotorHub;

class Motor {
  friend class MotorHub;

protected:
  Motor(mcpwm_cmpr_handle_t cmp, gpio_num_t A, gpio_num_t B);

public:
  // Full int16_t range, it will be divided to 2048 though.
  void set_direction(int16_t direction);

private:
  gpio_num_t A;
  gpio_num_t B;
  mcpwm_cmpr_handle_t cmp;
};
