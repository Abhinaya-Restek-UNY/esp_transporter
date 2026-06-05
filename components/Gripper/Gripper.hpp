#pragma once
#include "Servo.hpp"
#include "esp_timer.h"

class Gripper {
private:
  Servo *claw;
  Servo *lifter;
  bool claw_is_open = false;
  bool lifter_is_open = false;
  float current_angle = 0;
  float claw_step = 0.1;
  float lifted_angle = 0;
  float drop_angle = 0;

  uint32_t last_claw_toggle = this->get_time();
  uint32_t last_lifter_toggle = this->get_time();
  uint32_t debounce = 500;
  uint32_t get_time();

public:
  Gripper();
  Gripper(Servo *claw, Servo *lifter, uint32_t debounce);
  void set_claw(bool gripping);
  void set_lifter(int16_t throttlebreak);
};
