#pragma once
#include "PWMManager.hpp"
#include "driver/mcpwm_types.h"

class Servo {
  friend class ServoManager;

protected:
  Servo(mcpwm_cmpr_handle_t cmp, uint16_t max_angle, uint32_t min_tick,
        uint32_t max_tick);

public:
  void set_angle(float angle);
  uint16_t max_angle = 0;

private:
  mcpwm_cmpr_handle_t cmp;
  double scale = 0;
  uint32_t min_tick = 0;
};

class ServoManager : public PWMManager {
public:
  using PWMManager::PWMManager;
  Servo *create_servo(gpio_num_t PWM, uint16_t max_angle, float min_pulse_ms,
                      float max_pulse_ms);

private:
  ll<Servo> *last_serv = NULL;
};
