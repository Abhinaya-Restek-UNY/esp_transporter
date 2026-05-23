#pragma once
#include "driver/mcpwm_types.h"

class Servo {
  friend class MotorHub;

protected:
  Servo(mcpwm_cmpr_handle_t cmp, int16_t max_angle, int16_t min_tick,
        int16_t max_tick);

public:
  void set_angle(float angle);
  int16_t max_angle = 0;

private:
  mcpwm_cmpr_handle_t cmp;
  double scale = 0;
  int16_t min_tick = 0;
};
